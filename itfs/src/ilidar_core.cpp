/**
 * @file ilidar_core.cpp
 * @brief ROS 2 core node for iTFS LiDAR
 * @author Junwoo Son (json@hybo.co)
 * @date 2026-07-14
 * @version V2.0.0
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "ilidar.hpp"

namespace {
using namespace std::chrono_literals;

constexpr auto diagnostics_publish_period = 1s;
constexpr auto subscription_poll_period = 100ms;
constexpr int mapping_row = 240;
constexpr int mapping_col = 320;
constexpr double half_pi = 1.57079632679489661923;

constexpr uint8_t request_depth = 1u << 0;
constexpr uint8_t request_intensity = 1u << 1;
constexpr uint8_t request_pointcloud = 1u << 2;

struct FrameRequest {
    bool depth = false;
    bool intensity = false;
    bool pointcloud = false;

    bool any() const { return depth || intensity || pointcloud; }

    uint8_t mask() const {
        return (depth ? request_depth : 0) |
               (intensity ? request_intensity : 0) |
               (pointcloud ? request_pointcloud : 0);
    }

    static FrameRequest from_mask(uint8_t value) {
        FrameRequest request;
        request.depth = (value & request_depth) != 0;
        request.intensity = (value & request_intensity) != 0;
        request.pointcloud = (value & request_pointcloud) != 0;
        return request;
    }
};

struct MappingTable {
    std::array<float, mapping_row * mapping_col * 3> direction{};

    float at(int row, int col, int axis) const {
        return direction[(static_cast<size_t>(row) * mapping_col + col) * 3 + axis];
    }
};

struct ItfsFrameSnapshot {
    iTFS::img_t data{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint8_t request_mask = 0;
};

struct NormalizedInfo {
    bool valid = false;
    bool v2 = false;
    uint16_t sensor_sn = 0;
    uint8_t capture_mode = 0;
    uint8_t capture_row = iTFS::max_row;
    uint8_t data_output = 0;
    uint32_t capture_period_us = 0;
    uint8_t sensor_model_id = 0;
    uint8_t lock = 0;
    std::array<uint8_t, 3> firmware{};
    std::array<uint8_t, 4> sensor_ip{};
    std::array<uint8_t, 4> destination_ip{};
    uint16_t destination_port = 0;
};

struct DeviceSettings {
    std::string parent_frame_id = "base_link";
    std::string link_frame_id;
    std::string optical_frame_id;
    std::string mapping_file;
    bool publish_tf = true;
    bool publish_depth = true;
    bool publish_intensity = true;
    bool publish_pointcloud = true;
    bool publish_pointcloud_intensity = true;
    bool publish_info = true;
    bool publish_diagnostics = true;
    int sensor_qos_depth = 5;
    double mount_translation_x = 0.0;
    double mount_translation_y = 0.0;
    double mount_translation_z = 0.0;
    double mount_rotation_roll = 0.0;
    double mount_rotation_pitch = 0.0;
    double mount_rotation_yaw = 0.0;
    double optical_translation_x = 0.0;
    double optical_translation_y = 0.0;
    double optical_translation_z = 0.0;
    double optical_rotation_roll = -half_pi;
    double optical_rotation_pitch = 0.0;
    double optical_rotation_yaw = -half_pi;
};

struct DeviceContext {
    int device_idx = -1;
    uint16_t sensor_sn = 0;
    std::string topic_prefix;
    std::string parameter_prefix;
    NormalizedInfo info;
    DeviceSettings settings;
    std::shared_ptr<const MappingTable> mapping;

    std::mutex frame_mutex;
    std::condition_variable frame_cv;
    std::unique_ptr<ItfsFrameSnapshot> pending_frame = std::make_unique<ItfsFrameSnapshot>();
    std::unique_ptr<ItfsFrameSnapshot> working_frame = std::make_unique<ItfsFrameSnapshot>();
    bool frame_pending = false;
    bool worker_exit = false;
    std::thread worker_thread;
    std::atomic<uint8_t> subscription_mask{0};
    std::atomic<uint64_t> ros_frame_overwrite_count{0};
    std::atomic<bool> faulted{false};

    std::mutex status_mutex;
    iTFS::packet::status_t status{};
    iTFS::packet::status_full_t status_full{};
    bool status_received = false;
    uint64_t sensor_warning_frame_count = 0;
    uint64_t sensor_frame_warning_frame_count = 0;
    uint64_t image_frame_error_frame_count = 0;
    bool image_frame_error_latched = false;

    sensor_msgs::msg::Image depth_msg;
    sensor_msgs::msg::Image intensity_msg;
    sensor_msgs::msg::PointCloud2 points_msg;
    bool points_schema_initialized = false;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr intensity_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub;
};

template <typename T>
diagnostic_msgs::msg::KeyValue key_value(const std::string &key, const T &value) {
    diagnostic_msgs::msg::KeyValue result;
    std::ostringstream stream;
    stream << value;
    result.key = key;
    result.value = stream.str();
    return result;
}

bool parse_ipv4(const std::string &text, std::array<uint8_t, 4> &bytes) {
    unsigned int a = 0, b = 0, c = 0, d = 0;
    char tail = '\0';
    if (std::sscanf(text.c_str(), "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) != 4 ||
        a > 255 || b > 255 || c > 255 || d > 255) {
        return false;
    }
    bytes = {static_cast<uint8_t>(a), static_cast<uint8_t>(b),
             static_cast<uint8_t>(c), static_cast<uint8_t>(d)};
    return true;
}

std::string format_ipv4(const uint8_t ip[4]) {
    std::ostringstream stream;
    stream << static_cast<unsigned int>(ip[0]) << "."
           << static_cast<unsigned int>(ip[1]) << "."
           << static_cast<unsigned int>(ip[2]) << "."
           << static_cast<unsigned int>(ip[3]);
    return stream.str();
}

geometry_msgs::msg::TransformStamped make_transform(
    const rclcpp::Time &stamp, const std::string &parent, const std::string &child,
    double tx, double ty, double tz, double roll, double pitch, double yaw) {
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    quaternion.normalize();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.transform.translation.x = tx;
    transform.transform.translation.y = ty;
    transform.transform.translation.z = tz;
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();
    return transform;
}

NormalizedInfo normalize_info(const iTFS::device_t &device) {
    NormalizedInfo result;
    if (device.info_v2.sensor_sn != 0) {
        const auto &info = device.info_v2;
        result.valid = true;
        result.v2 = true;
        result.sensor_sn = info.sensor_sn;
        result.capture_mode = info.capture_mode;
        result.capture_row = info.capture_row;
        result.data_output = info.data_output;
        result.capture_period_us = info.capture_period_us;
        result.sensor_model_id = info.sensor_model_id;
        result.lock = info.lock;
        std::copy_n(info.sensor_fw_ver, 3, result.firmware.begin());
        std::copy_n(info.data_sensor_ip, 4, result.sensor_ip.begin());
        std::copy_n(info.data_dest_ip, 4, result.destination_ip.begin());
        result.destination_port = info.data_port;
    } else if (device.info.sensor_sn != 0) {
        const auto &info = device.info;
        result.valid = true;
        result.sensor_sn = info.sensor_sn;
        result.capture_mode = info.capture_mode;
        result.capture_row = info.capture_row;
        result.data_output = info.data_output;
        result.capture_period_us = static_cast<uint32_t>(info.capture_period) * 1000u;
        result.lock = info.lock;
        std::copy_n(info.sensor_fw_ver, 3, result.firmware.begin());
        std::copy_n(info.data_sensor_ip, 4, result.sensor_ip.begin());
        std::copy_n(info.data_dest_ip, 4, result.destination_ip.begin());
        result.destination_port = info.data_port;
    }
    return result;
}
} // namespace

class ItfsCoreNode;
static std::atomic<ItfsCoreNode *> g_node{nullptr};

class ItfsCoreNode : public rclcpp::Node {
  public:
    ItfsCoreNode() : Node("ilidar_core") {
        for (auto &context : context_by_idx_) context.store(nullptr);
        for (auto &failed : context_init_failed_) failed.store(false);
        declare_parameters();
        if (iTFS::version() != 0) {
            throw std::runtime_error("iTFS API and packet versions do not match");
        }

        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(diagnostics_qos_depth_)).reliable());
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        diagnostics_timer_ = create_wall_timer(diagnostics_publish_period,
                                                [this] { publish_diagnostics(); });
        subscription_timer_ = create_wall_timer(subscription_poll_period,
                                                 [this] { update_subscription_masks(); });

        uint8_t *broadcast_ptr = optional_ip(broadcast_ip_, broadcast_ip_bytes_, "broadcast_ip");
        uint8_t *listening_ptr = optional_ip(listening_ip_, listening_ip_bytes_, "listening_ip");
        g_node.store(this, std::memory_order_release);
        lidar_ = std::make_unique<iTFS::LiDAR>(
            &ItfsCoreNode::lidar_data_handler,
            &ItfsCoreNode::status_packet_handler,
            &ItfsCoreNode::info_packet_handler,
            broadcast_ptr, listening_ptr, static_cast<uint16_t>(listening_port_));

        while (rclcpp::ok() && !lidar_->Ready()) std::this_thread::sleep_for(100ms);
        if (!rclcpp::ok()) {
            g_node.store(nullptr, std::memory_order_release);
            lidar_->Try_exit();
            lidar_->Join();
            throw std::runtime_error("ROS shutdown while waiting for iTFS::LiDAR");
        }
        RCLCPP_INFO(get_logger(), "iTFS::LiDAR is ready. Waiting for up to %d devices.",
                    iTFS::max_device);
    }

    ~ItfsCoreNode() override {
        g_node.store(nullptr, std::memory_order_release);
        if (lidar_) {
            lidar_->Try_exit();
            lidar_->Join();
        }
        for (auto &owned : contexts_) {
            if (!owned) continue;
            {
                std::lock_guard<std::mutex> lock(owned->frame_mutex);
                owned->worker_exit = true;
                owned->frame_cv.notify_all();
            }
            if (owned->worker_thread.joinable()) owned->worker_thread.join();
        }
    }

  private:
    template <typename T>
    T immutable_parameter(const std::string &name, const T &default_value) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        descriptor.description = "Fixed at startup; restart to apply a new value";
        return declare_parameter<T>(name, default_value, descriptor);
    }

    void declare_parameters() {
        broadcast_ip_ = immutable_parameter<std::string>("broadcast_ip", "");
        listening_ip_ = immutable_parameter<std::string>("listening_ip", "");
        listening_port_ = immutable_parameter<int>("listening_port", iTFS::user_data_port);
        default_mapping_file_ = ament_index_cpp::get_package_share_directory("ilidar_itfs_ros2") +
                                "/dat/iTFS-110.dat";
        defaults_.mapping_file = immutable_parameter<std::string>("mapping_file", default_mapping_file_);
        defaults_.parent_frame_id = immutable_parameter<std::string>("parent_frame_id", "base_link");
        defaults_.publish_tf = immutable_parameter<bool>("publish_tf", true);
        defaults_.publish_depth = immutable_parameter<bool>("publish_depth", true);
        defaults_.publish_intensity = immutable_parameter<bool>("publish_intensity", true);
        defaults_.publish_pointcloud = immutable_parameter<bool>("publish_pointcloud", true);
        defaults_.publish_pointcloud_intensity =
            immutable_parameter<bool>("publish_pointcloud_intensity", true);
        defaults_.publish_info = immutable_parameter<bool>("publish_info", true);
        defaults_.publish_diagnostics = immutable_parameter<bool>("publish_diagnostics", true);
        defaults_.sensor_qos_depth = static_cast<int>(std::max<int64_t>(
            immutable_parameter<int64_t>("sensor_qos_depth", 5), 1));
        diagnostics_qos_depth_ = static_cast<int>(std::max<int64_t>(
            immutable_parameter<int64_t>("diagnostics_qos_depth", 10), 1));
        defaults_.mount_translation_x = immutable_parameter<double>("mount_translation_x", 0.0);
        defaults_.mount_translation_y = immutable_parameter<double>("mount_translation_y", 0.0);
        defaults_.mount_translation_z = immutable_parameter<double>("mount_translation_z", 0.0);
        defaults_.mount_rotation_roll = immutable_parameter<double>("mount_rotation_roll", 0.0);
        defaults_.mount_rotation_pitch = immutable_parameter<double>("mount_rotation_pitch", 0.0);
        defaults_.mount_rotation_yaw = immutable_parameter<double>("mount_rotation_yaw", 0.0);
        defaults_.optical_translation_x = immutable_parameter<double>("optical_translation_x", 0.0);
        defaults_.optical_translation_y = immutable_parameter<double>("optical_translation_y", 0.0);
        defaults_.optical_translation_z = immutable_parameter<double>("optical_translation_z", 0.0);
        defaults_.optical_rotation_roll = immutable_parameter<double>("optical_rotation_roll", -half_pi);
        defaults_.optical_rotation_pitch = immutable_parameter<double>("optical_rotation_pitch", 0.0);
        defaults_.optical_rotation_yaw = immutable_parameter<double>("optical_rotation_yaw", -half_pi);
        if (listening_port_ < 1 || listening_port_ > 65535) {
            throw std::invalid_argument("listening_port must be in the range 1..65535");
        }
    }

    uint8_t *optional_ip(const std::string &text, std::array<uint8_t, 4> &storage,
                         const char *name) {
        if (text.empty()) return nullptr;
        if (!parse_ipv4(text, storage)) {
            throw std::invalid_argument(std::string(name) + " must be an IPv4 address");
        }
        return storage.data();
    }

    template <typename T>
    T device_parameter(const DeviceContext &context, const std::string &name,
                       const T &default_value) {
        return immutable_parameter<T>(context.parameter_prefix + name, default_value);
    }

    DeviceSettings load_device_settings(DeviceContext &context) {
        DeviceSettings settings = defaults_;
        const std::string identity = "ilidar_" + std::to_string(context.sensor_sn);
        context.parameter_prefix = "devices." + identity + ".";
        settings.parent_frame_id = device_parameter(context, "parent_frame_id", defaults_.parent_frame_id);
        settings.link_frame_id = device_parameter(context, "link_frame_id", identity + "_link");
        settings.optical_frame_id = device_parameter(context, "frame_id", identity + "_optical_frame");
        settings.mapping_file = device_parameter(context, "mapping_file", defaults_.mapping_file);
        settings.publish_tf = device_parameter(context, "publish_tf", defaults_.publish_tf);
        settings.publish_depth = device_parameter(context, "publish_depth", defaults_.publish_depth);
        settings.publish_intensity = device_parameter(context, "publish_intensity", defaults_.publish_intensity);
        settings.publish_pointcloud = device_parameter(context, "publish_pointcloud", defaults_.publish_pointcloud);
        settings.publish_pointcloud_intensity = device_parameter(
            context, "publish_pointcloud_intensity", defaults_.publish_pointcloud_intensity);
        settings.publish_info = device_parameter(context, "publish_info", defaults_.publish_info);
        settings.publish_diagnostics = device_parameter(
            context, "publish_diagnostics", defaults_.publish_diagnostics);
        settings.sensor_qos_depth = static_cast<int>(std::max<int64_t>(device_parameter<int64_t>(
            context, "sensor_qos_depth", defaults_.sensor_qos_depth), 1));
#define LOAD_POSE(name) settings.name = device_parameter(context, #name, defaults_.name)
        LOAD_POSE(mount_translation_x); LOAD_POSE(mount_translation_y); LOAD_POSE(mount_translation_z);
        LOAD_POSE(mount_rotation_roll); LOAD_POSE(mount_rotation_pitch); LOAD_POSE(mount_rotation_yaw);
        LOAD_POSE(optical_translation_x); LOAD_POSE(optical_translation_y); LOAD_POSE(optical_translation_z);
        LOAD_POSE(optical_rotation_roll); LOAD_POSE(optical_rotation_pitch); LOAD_POSE(optical_rotation_yaw);
#undef LOAD_POSE
        return settings;
    }

    std::shared_ptr<const MappingTable> load_mapping(const std::string &path) {
        std::lock_guard<std::mutex> lock(mapping_mutex_);
        auto found = mapping_cache_.find(path);
        if (found != mapping_cache_.end()) return found->second;
        auto mapping = std::make_shared<MappingTable>();
        std::ifstream input(path, std::ios::binary);
        if (!input.read(reinterpret_cast<char *>(mapping->direction.data()),
                        static_cast<std::streamsize>(mapping->direction.size() * sizeof(float)))) {
            throw std::runtime_error("failed to read iTFS mapping file: " + path);
        }
        mapping_cache_[path] = mapping;
        return mapping;
    }

    static void validate_frame_id(const std::string &frame_id, const char *name) {
        if (frame_id.empty() || frame_id.front() == '/' ||
            std::any_of(frame_id.begin(), frame_id.end(), [](unsigned char value) {
                return std::isspace(value) != 0;
            })) {
            throw std::invalid_argument(std::string(name) + " is not a valid TF frame ID");
        }
    }

    void validate_device_frames(const DeviceContext &candidate) const {
        validate_frame_id(candidate.settings.parent_frame_id, "parent_frame_id");
        validate_frame_id(candidate.settings.link_frame_id, "link_frame_id");
        validate_frame_id(candidate.settings.optical_frame_id, "frame_id");
        if (candidate.settings.parent_frame_id == candidate.settings.link_frame_id ||
            candidate.settings.parent_frame_id == candidate.settings.optical_frame_id ||
            candidate.settings.link_frame_id == candidate.settings.optical_frame_id) {
            throw std::invalid_argument("parent, link, and optical frames must be distinct");
        }
        const double pose_values[] = {
            candidate.settings.mount_translation_x, candidate.settings.mount_translation_y,
            candidate.settings.mount_translation_z, candidate.settings.mount_rotation_roll,
            candidate.settings.mount_rotation_pitch, candidate.settings.mount_rotation_yaw,
            candidate.settings.optical_translation_x, candidate.settings.optical_translation_y,
            candidate.settings.optical_translation_z, candidate.settings.optical_rotation_roll,
            candidate.settings.optical_rotation_pitch, candidate.settings.optical_rotation_yaw
        };
        if (std::any_of(std::begin(pose_values), std::end(pose_values),
                        [](double value) { return !std::isfinite(value); })) {
            throw std::invalid_argument("TF translation and rotation parameters must be finite");
        }
        for (const auto &other : contexts_) {
            if (!other) continue;
            if (candidate.settings.link_frame_id == other->settings.link_frame_id ||
                candidate.settings.link_frame_id == other->settings.optical_frame_id ||
                candidate.settings.optical_frame_id == other->settings.link_frame_id ||
                candidate.settings.optical_frame_id == other->settings.optical_frame_id) {
                throw std::invalid_argument("child TF frame IDs must be unique across sensors");
            }
        }
    }

    void create_publishers(DeviceContext &context) {
        rclcpp::SensorDataQoS qos;
        qos.keep_last(context.settings.sensor_qos_depth);
        if (context.settings.publish_depth) {
            context.depth_pub = create_publisher<sensor_msgs::msg::Image>(
                context.topic_prefix + "/depth/image_raw", qos);
        }
        if (context.settings.publish_intensity) {
            context.intensity_pub = create_publisher<sensor_msgs::msg::Image>(
                context.topic_prefix + "/intensity/image_raw", qos);
        }
        if (context.settings.publish_pointcloud) {
            context.points_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
                context.topic_prefix + "/points", qos);
        }
        if (context.settings.publish_info) {
            rclcpp::QoS info_qos(rclcpp::KeepLast(1));
            info_qos.reliable().transient_local();
            context.info_pub = create_publisher<std_msgs::msg::String>(
                context.topic_prefix + "/info", info_qos);
        }
    }

    DeviceContext *context_for(int index) const {
        if (index < 0 || index >= iTFS::max_device) return nullptr;
        return context_by_idx_[index].load(std::memory_order_acquire);
    }

    DeviceContext *initialize_context(iTFS::device_t *device, const NormalizedInfo &info) {
        if (device->idx < 0 || device->idx >= iTFS::max_device || !info.valid) return nullptr;
        if (context_init_failed_[device->idx].load()) return nullptr;
        std::lock_guard<std::mutex> lock(context_init_mutex_);
        if (DeviceContext *existing = context_for(device->idx)) return existing;
        for (const auto &other : contexts_) {
            if (other && other->sensor_sn == info.sensor_sn) {
                RCLCPP_ERROR(get_logger(), "Duplicate sensor SN#%u.", info.sensor_sn);
                context_init_failed_[device->idx].store(true);
                return nullptr;
            }
        }
        try {
            if (info.capture_mode > 3 || info.capture_row < 4 || info.capture_row > iTFS::max_row) {
                throw std::invalid_argument("invalid capture mode or row count");
            }
            auto owned = std::make_unique<DeviceContext>();
            DeviceContext *context = owned.get();
            context->device_idx = device->idx;
            context->sensor_sn = info.sensor_sn;
            context->info = info;
            context->topic_prefix = "/ilidar_" + std::to_string(info.sensor_sn);
            context->settings = load_device_settings(*context);
            validate_device_frames(*context);
            if (context->settings.publish_pointcloud) {
                try {
                    context->mapping = load_mapping(context->settings.mapping_file);
                } catch (const std::exception &error) {
                    RCLCPP_ERROR(get_logger(), "%s PointCloud2 will remain unavailable.", error.what());
                }
            }
            create_publishers(*context);
            if (context->settings.publish_tf) publish_static_tf(*context);
            context->worker_thread = std::thread(&ItfsCoreNode::frame_worker, this, context);
            contexts_[device->idx] = std::move(owned);
            context_by_idx_[device->idx].store(context, std::memory_order_release);
            publish_info(*context);
            RCLCPP_INFO(get_logger(), "D#%d SN#%u initialized at %s.", device->idx,
                        info.sensor_sn, context->topic_prefix.c_str());
            return context;
        } catch (const std::exception &error) {
            context_init_failed_[device->idx].store(true);
            RCLCPP_ERROR(get_logger(), "D#%d SN#%u initialization failed: %s",
                         device->idx, info.sensor_sn, error.what());
            return nullptr;
        }
    }

    template <typename Publisher>
    bool has_subscribers(const Publisher &publisher) const {
        return publisher && (publisher->get_subscription_count() > 0 ||
                             publisher->get_intra_process_subscription_count() > 0);
    }

    void update_subscription_masks() {
        for (int index = 0; index < iTFS::max_device; ++index) {
            if (DeviceContext *context = context_for(index)) {
                FrameRequest request;
                request.depth = context->info.capture_mode != 0 && has_subscribers(context->depth_pub);
                request.intensity = has_subscribers(context->intensity_pub);
                request.pointcloud = context->info.capture_mode != 0 &&
                                     has_subscribers(context->points_pub);
                context->subscription_mask.store(request.mask(), std::memory_order_relaxed);
            }
        }
    }

    static void lidar_data_handler(iTFS::device_t *device) {
        if (ItfsCoreNode *node = g_node.load(std::memory_order_acquire)) node->on_data(device);
    }
    static void status_packet_handler(iTFS::device_t *device) {
        if (ItfsCoreNode *node = g_node.load(std::memory_order_acquire)) node->on_status(device);
    }
    static void info_packet_handler(iTFS::device_t *device) {
        if (ItfsCoreNode *node = g_node.load(std::memory_order_acquire)) node->on_info(device);
    }

    void on_info(iTFS::device_t *device) {
        const NormalizedInfo info = normalize_info(*device);
        if (!info.valid) return;
        DeviceContext *context = context_for(device->idx);
        if (!context) context = initialize_context(device, info);
        if (!context) return;
        if (context->sensor_sn != info.sensor_sn ||
            context->info.capture_mode != info.capture_mode ||
            context->info.capture_row != info.capture_row ||
            context->info.data_output != info.data_output ||
            context->info.sensor_model_id != info.sensor_model_id) {
            context->faulted.store(true);
            RCLCPP_ERROR(get_logger(),
                "D#%d SN/config changed while running; publication is disabled until restart.",
                device->idx);
        }
    }

    void on_status(iTFS::device_t *device) {
        DeviceContext *context = context_for(device->idx);
        if (!context || context->faulted.load()) return;
        std::lock_guard<std::mutex> lock(context->status_mutex);
        context->status = device->status;
        context->status_full = device->status_full;
        context->status_received = true;
        if (device->status.sensor_warning != 0) ++context->sensor_warning_frame_count;
        if (device->status.sensor_frame_status != 0) ++context->sensor_frame_warning_frame_count;
    }

    void on_data(iTFS::device_t *device) {
        DeviceContext *context = context_for(device->idx);
        if (!context || context->faulted.load()) return;
        const FrameRequest request = FrameRequest::from_mask(
            context->subscription_mask.load(std::memory_order_relaxed));
        if (!request.any()) return;
        if (device->data.mode != context->info.capture_mode ||
            device->data.capture_row != context->info.capture_row) {
            context->faulted.store(true);
            RCLCPP_ERROR(get_logger(),
                         "D#%d capture mode/row changed while running.", device->idx);
            return;
        }

        std::lock_guard<std::mutex> lock(context->frame_mutex);
        auto &frame = *context->pending_frame;
        frame.data.mode = device->data.mode;
        frame.data.frame = device->data.frame;
        frame.data.capture_row = device->data.capture_row;
        frame.data.frame_status = device->data.frame_status;
        const int height = device->data.mode == 0 ? iTFS::gray_row : context->info.capture_row;
        if (device->data.mode == 0) {
            if (request.intensity) {
                std::memcpy(frame.data.img, device->data.img,
                            static_cast<size_t>(height) * iTFS::max_col * sizeof(uint16_t));
            }
        } else {
            if (request.depth || request.pointcloud) {
                std::memcpy(frame.data.img, device->data.img,
                            static_cast<size_t>(height) * iTFS::max_col * sizeof(uint16_t));
            }
            const bool intensity_available =
                (context->info.data_output & iTFS::packet::data_output_intensity_mask) != 0;
            if (intensity_available &&
                (request.intensity || (request.pointcloud &&
                                       context->settings.publish_pointcloud_intensity))) {
                std::memcpy(&frame.data.img[height], &device->data.img[height],
                            static_cast<size_t>(height) * iTFS::max_col * sizeof(uint16_t));
            }
        }
        frame.stamp = now();
        frame.request_mask = request.mask();
        if (context->frame_pending) ++context->ros_frame_overwrite_count;
        context->frame_pending = true;
        if (device->data.frame_status != iTFS::status_normal) {
            std::lock_guard<std::mutex> status_lock(context->status_mutex);
            ++context->image_frame_error_frame_count;
            context->image_frame_error_latched = true;
        }
        context->frame_cv.notify_one();
    }

    void frame_worker(DeviceContext *context) {
        while (true) {
            {
                std::unique_lock<std::mutex> lock(context->frame_mutex);
                context->frame_cv.wait(lock, [&] { return context->worker_exit || context->frame_pending; });
                if (context->worker_exit) return;
                std::swap(context->pending_frame, context->working_frame);
                context->frame_pending = false;
            }
            publish_frame(*context, *context->working_frame);
        }
    }

    void prepare_image(sensor_msgs::msg::Image &message, int height,
                       const std::string &frame_id) const {
        message.header.frame_id = frame_id;
        message.height = static_cast<uint32_t>(height);
        message.width = iTFS::max_col;
        message.encoding = "mono16";
        message.is_bigendian = false;
        message.step = iTFS::max_col * sizeof(uint16_t);
        message.data.resize(static_cast<size_t>(height) * message.step);
    }

    void copy_image(const uint16_t *source, sensor_msgs::msg::Image &message) const {
        for (size_t index = 0; index < message.data.size() / 2; ++index) {
            const uint16_t value = source[index];
            message.data[index * 2] = static_cast<uint8_t>(value & 0xff);
            message.data[index * 2 + 1] = static_cast<uint8_t>(value >> 8);
        }
    }

    void publish_frame(DeviceContext &context, const ItfsFrameSnapshot &frame) {
        if (context.faulted.load()) return;
        const FrameRequest request = FrameRequest::from_mask(frame.request_mask);
        if (frame.data.mode == 0) {
            if (request.intensity && context.intensity_pub) {
                prepare_image(context.intensity_msg, iTFS::gray_row,
                              context.settings.optical_frame_id);
                copy_image(&frame.data.img[0][0], context.intensity_msg);
                context.intensity_msg.header.stamp = frame.stamp;
                context.intensity_pub->publish(context.intensity_msg);
            }
            return;
        }

        const int height = context.info.capture_row;
        const bool depth_available =
            (context.info.data_output & iTFS::packet::data_output_depth_mask) != 0;
        const bool intensity_available =
            (context.info.data_output & iTFS::packet::data_output_intensity_mask) != 0;
        if (request.depth && depth_available && context.depth_pub) {
            prepare_image(context.depth_msg, height, context.settings.optical_frame_id);
            copy_image(&frame.data.img[0][0], context.depth_msg);
            context.depth_msg.header.stamp = frame.stamp;
            context.depth_pub->publish(context.depth_msg);
        }
        if (request.intensity && intensity_available && context.intensity_pub) {
            prepare_image(context.intensity_msg, height, context.settings.optical_frame_id);
            copy_image(&frame.data.img[height][0], context.intensity_msg);
            context.intensity_msg.header.stamp = frame.stamp;
            context.intensity_pub->publish(context.intensity_msg);
        }
        if (request.pointcloud && depth_available && context.points_pub && context.mapping) {
            publish_pointcloud(context, frame, height, intensity_available);
        }
    }

    void publish_pointcloud(DeviceContext &context, const ItfsFrameSnapshot &frame,
                            int height, bool intensity_available) {
        const bool include_intensity = intensity_available &&
                                       context.settings.publish_pointcloud_intensity;
        auto &cloud = context.points_msg;
        cloud.header.stamp = frame.stamp;
        cloud.header.frame_id = context.settings.link_frame_id;
        cloud.height = height;
        cloud.width = iTFS::max_col;
        cloud.is_dense = false;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        if (!context.points_schema_initialized) {
            if (include_intensity) {
                modifier.setPointCloud2Fields(
                    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
            } else {
                modifier.setPointCloud2FieldsByString(1, "xyz");
            }
            context.points_schema_initialized = true;
        }
        modifier.resize(static_cast<size_t>(height) * iTFS::max_col);
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
        std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> intensity;
        if (include_intensity) {
            intensity = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(cloud, "intensity");
        }
        const int mapping_offset = (mapping_row - height) / 2;
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < iTFS::max_col; ++col, ++x, ++y, ++z) {
                const uint16_t depth_mm = frame.data.img[row][col];
                if (depth_mm == 0) {
                    const float nan = std::numeric_limits<float>::quiet_NaN();
                    *x = nan; *y = nan; *z = nan;
                } else {
                    const float depth_m = static_cast<float>(depth_mm) * 0.001f;
                    const int map_row = row + mapping_offset;
                    *x = depth_m * context.mapping->at(map_row, col, 2);
                    *y = -depth_m * context.mapping->at(map_row, col, 0);
                    *z = -depth_m * context.mapping->at(map_row, col, 1);
                }
                if (intensity) {
                    **intensity = static_cast<float>(frame.data.img[height + row][col]);
                    ++(*intensity);
                }
            }
        }
        context.points_pub->publish(cloud);
    }

    void publish_info(DeviceContext &context) {
        if (!context.info_pub) return;
        const auto &info = context.info;
        std::ostringstream stream;
        stream << "SN# " << info.sensor_sn << " D# " << context.device_idx
               << " LOCK " << static_cast<unsigned int>(info.lock)
               << " CAPTURE_MODE " << static_cast<unsigned int>(info.capture_mode)
               << " CAPTURE_ROW " << static_cast<unsigned int>(info.capture_row)
               << " DATA_OUTPUT 0x" << std::hex << static_cast<unsigned int>(info.data_output)
               << std::dec << " PERIOD " << info.capture_period_us << " us"
               << " MODEL_ID " << static_cast<unsigned int>(info.sensor_model_id)
               << " SENSOR_IP " << format_ipv4(info.sensor_ip.data())
               << " DEST " << format_ipv4(info.destination_ip.data()) << ":" << info.destination_port
               << " FW V" << static_cast<unsigned int>(info.firmware[2]) << "."
               << static_cast<unsigned int>(info.firmware[1]) << "."
               << static_cast<unsigned int>(info.firmware[0])
               << " TOPIC " << context.topic_prefix;
        std_msgs::msg::String message;
        message.data = stream.str();
        context.info_pub->publish(message);
        RCLCPP_INFO(get_logger(), "%s", message.data.c_str());
    }

    void publish_diagnostics() {
        if (!diagnostics_pub_) return;
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = now();
        for (int index = 0; index < iTFS::max_device; ++index) {
            DeviceContext *context = context_for(index);
            if (!context || !context->settings.publish_diagnostics) continue;
            iTFS::packet::status_t status{};
            bool status_received = false;
            bool image_error_latched = false;
            uint64_t warning_count = 0, frame_warning_count = 0, image_error_count = 0;
            {
                std::lock_guard<std::mutex> lock(context->status_mutex);
                status = context->status;
                status_received = context->status_received;
                warning_count = context->sensor_warning_frame_count;
                frame_warning_count = context->sensor_frame_warning_frame_count;
                image_error_count = context->image_frame_error_frame_count;
                image_error_latched = context->image_frame_error_latched;
                context->image_frame_error_latched = false;
            }
            diagnostic_msgs::msg::DiagnosticStatus diagnostic;
            diagnostic.name = "ilidar_" + std::to_string(context->sensor_sn) + "/status";
            diagnostic.hardware_id = std::to_string(context->sensor_sn);
            if (context->faulted.load()) {
                diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                diagnostic.message = "sensor identity or immutable capture setting changed";
            } else if (!status_received || status.sensor_warning != 0 ||
                       status.sensor_frame_status != 0 || image_error_latched ||
                       (context->settings.publish_pointcloud && !context->mapping)) {
                diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                diagnostic.message = !status_received ? "waiting for status" : "sensor/frame warning";
            } else {
                diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                diagnostic.message = "OK";
            }
            diagnostic.values = {
                key_value("device_index", context->device_idx),
                key_value("sensor_sn", context->sensor_sn),
                key_value("topic_prefix", context->topic_prefix),
                key_value("capture_mode", static_cast<unsigned int>(context->info.capture_mode)),
                key_value("capture_row", static_cast<unsigned int>(context->info.capture_row)),
                key_value("data_output", static_cast<unsigned int>(context->info.data_output)),
                key_value("sensor_time_us", iTFS::packet::get_sensor_time_in_us(&status)),
                key_value("capture_frame", static_cast<unsigned int>(status.capture_frame)),
                key_value("sensor_frame_status", status.sensor_frame_status),
                key_value("sensor_warning", status.sensor_warning),
                key_value("sensor_temp_rx_c", static_cast<double>(status.sensor_temp_rx) * 0.01),
                key_value("sensor_temp_core_c", static_cast<double>(status.sensor_temp_core) * 0.01),
                key_value("sensor_vcsel_level", status.sensor_vcsel_level),
                key_value("sensor_power_level", status.sensor_power_level),
                key_value("sensor_warning_frame_count", warning_count),
                key_value("sensor_frame_warning_frame_count", frame_warning_count),
                key_value("image_frame_error_frame_count", image_error_count),
                key_value("ros_frame_overwrite_count", context->ros_frame_overwrite_count.load()),
                key_value("mapping_file", context->settings.mapping_file),
                key_value("parent_frame_id", context->settings.parent_frame_id),
                key_value("link_frame_id", context->settings.link_frame_id),
                key_value("optical_frame_id", context->settings.optical_frame_id)
            };
            array.status.push_back(std::move(diagnostic));
        }
        if (!array.status.empty()) diagnostics_pub_->publish(array);
    }

    void publish_static_tf(const DeviceContext &context) {
        const rclcpp::Time stamp = now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(make_transform(
            stamp, context.settings.parent_frame_id, context.settings.link_frame_id,
            context.settings.mount_translation_x, context.settings.mount_translation_y,
            context.settings.mount_translation_z, context.settings.mount_rotation_roll,
            context.settings.mount_rotation_pitch, context.settings.mount_rotation_yaw));
        transforms.push_back(make_transform(
            stamp, context.settings.link_frame_id, context.settings.optical_frame_id,
            context.settings.optical_translation_x, context.settings.optical_translation_y,
            context.settings.optical_translation_z, context.settings.optical_rotation_roll,
            context.settings.optical_rotation_pitch, context.settings.optical_rotation_yaw));
        static_tf_broadcaster_->sendTransform(transforms);
    }

    std::unique_ptr<iTFS::LiDAR> lidar_;
    std::array<std::unique_ptr<DeviceContext>, iTFS::max_device> contexts_;
    std::array<std::atomic<DeviceContext *>, iTFS::max_device> context_by_idx_;
    std::array<std::atomic<bool>, iTFS::max_device> context_init_failed_;
    std::mutex context_init_mutex_;
    std::mutex mapping_mutex_;
    std::unordered_map<std::string, std::shared_ptr<const MappingTable>> mapping_cache_;
    DeviceSettings defaults_;
    std::string default_mapping_file_;
    std::string broadcast_ip_;
    std::string listening_ip_;
    int listening_port_ = iTFS::user_data_port;
    int diagnostics_qos_depth_ = 10;
    std::array<uint8_t, 4> broadcast_ip_bytes_{};
    std::array<uint8_t, 4> listening_ip_bytes_{};
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::TimerBase::SharedPtr subscription_timer_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ItfsCoreNode>());
    } catch (const std::exception &error) {
        std::fprintf(stderr, "[ERROR] ilidar_core: %s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
