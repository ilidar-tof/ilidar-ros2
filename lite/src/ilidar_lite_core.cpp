/**
 * @file ilidar_lite_core.cpp
 * @brief ROS 2 core node for iTFS-LITE
 * @author Junwoo Son (json@hybo.co)
 * @date 2026-07-13
 * @version V2.0.1
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "ilidar_lite.hpp"

namespace {
// Keep the range constants identical to lite_pcl_example.cpp so every SDK
// encoding is reconstructed with the same physical scale.
constexpr double half_pi = 1.5707963267948966;
constexpr float depth_q16_max_m = 7.49481145f;
constexpr float depth_raw_q16_to_m = depth_q16_max_m / 65535.0f;
constexpr float depth_lin8_to_m = depth_q16_max_m / 255.0f;
constexpr float xyz_raw_q15_to_m = depth_q16_max_m / 32767.0f;
constexpr float xyz_lin8_to_m = depth_q16_max_m / 127.0f;
constexpr uint16_t confidence_raw_valid_threshold = 16;
constexpr auto output_mode_warning_repeat_period = std::chrono::seconds(5);
constexpr auto diagnostics_publish_period = std::chrono::seconds(1);
constexpr auto subscription_poll_period = std::chrono::milliseconds(100);

constexpr uint8_t request_depth = 1u << 0;
constexpr uint8_t request_amplitude = 1u << 1;
constexpr uint8_t request_intensity = 1u << 2;
constexpr uint8_t request_confidence = 1u << 3;
constexpr uint8_t request_camera_info = 1u << 4;
constexpr uint8_t request_pointcloud = 1u << 5;

// Owns one completed SDK frame after its active packed slots have been copied.
// The ROS timestamp and calibration snapshot keep all products from this frame
// mutually consistent even while the SDK starts receiving the next frame.
struct LiteFrameSnapshot {
    iTFS::lite_img_cpy_t lite_img_data{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint8_t request_mask = 0;
    bool calibration_ready = false;
};

// Subscriber demand is polled by a ROS timer and encoded into one byte. The
// same byte is stored with the frame so copying and publication cannot disagree
// if subscriptions change while the worker is processing.
struct FrameRequest {
    bool depth = false;
    bool amplitude = false;
    bool intensity = false;
    bool confidence = false;
    bool camera_info = false;
    bool pointcloud = false;

    // A zero mask lets the SDK callback return without copying image slots.
    bool any() const {
        return depth || amplitude || intensity || confidence || camera_info || pointcloud;
    }

    uint8_t mask() const {
        return (depth ? request_depth : 0) |
               (amplitude ? request_amplitude : 0) |
               (intensity ? request_intensity : 0) |
               (confidence ? request_confidence : 0) |
               (camera_info ? request_camera_info : 0) |
               (pointcloud ? request_pointcloud : 0);
    }

    // Reconstruct the immutable per-frame request captured by the callback.
    static FrameRequest from_mask(uint8_t value) {
        FrameRequest request;
        request.depth = (value & request_depth) != 0;
        request.amplitude = (value & request_amplitude) != 0;
        request.intensity = (value & request_intensity) != 0;
        request.confidence = (value & request_confidence) != 0;
        request.camera_info = (value & request_camera_info) != 0;
        request.pointcloud = (value & request_pointcloud) != 0;
        return request;
    }
};

struct LiteStatusEvent {
    iTFS::packet::status_v3_t status{};
    int image_frame_status = 0;
    bool valid = false;
};

// The SDK callback keeps the latest status plus interval latches and cumulative
// affected-frame counts. Fault snapshots preserve the frame/time that produced
// each diagnostic condition instead of attributing it to the newest good frame.
struct LiteStatusSnapshot {
    iTFS::packet::status_v3_t status{};
    int image_frame_status = 0;
    uint32_t pending_sensor_warning = 0;
    uint16_t pending_sensor_frame_status = 0;
    int pending_image_frame_status = 0;
    LiteStatusEvent sensor_warning_event;
    LiteStatusEvent sensor_frame_warning_event;
    LiteStatusEvent image_frame_error_event;
    uint64_t status_packet_count = 0;
    uint64_t sensor_warning_frame_count = 0;
    uint64_t sensor_frame_warning_frame_count = 0;
    uint64_t image_frame_error_frame_count = 0;
};

// Immutable data_output layout decoded once from the first info_v3 packet.
// Numeric modes drive SDK slot access; names and warning drive ROS reporting.
struct LiteOutputLayout {
    uint16_t data_output = 0;
    uint16_t depth_mode = iTFS::packet::info_v3_data_output_depth_off;
    uint16_t amplitude_mode = iTFS::packet::info_v3_data_output_stream_off;
    uint16_t intensity_mode = iTFS::packet::info_v3_data_output_stream_off;
    uint16_t confidence_mode = iTFS::packet::info_v3_data_output_stream_off;
    bool depth_8bit = false;
    bool amplitude_8bit = false;
    bool intensity_8bit = false;
    bool confidence_mask1 = false;
    const char *depth_name = "UNKNOWN";
    const char *amplitude_name = "UNKNOWN";
    const char *intensity_name = "UNKNOWN";
    const char *confidence_name = "UNKNOWN";
    std::string warning;

    // Native ROS modes can be copied directly; every other received mode
    // requires restoration or unit conversion in the worker.
    bool conversion_required() const {
        return !warning.empty();
    }
};

struct DeviceSettings {
    std::string parent_frame_id = "base_link";
    std::string link_frame_id;
    std::string optical_frame_id;
    bool publish_tf = true;
    bool publish_depth = true;
    bool publish_amplitude = true;
    bool publish_intensity = true;
    bool publish_confidence = true;
    bool publish_camera_info = true;
    bool publish_pointcloud = true;
    bool publish_pointcloud_amplitude = true;
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

// All high-rate state is isolated per sensor. Contexts are allocated lazily
// after a valid serial number arrives, so disconnected SDK slots cost nothing.
struct DeviceContext {
    explicit DeviceContext(int index, uint16_t serial)
        : device_idx(index), sensor_sn(serial),
          topic_prefix("/ilidar_lite_" + std::to_string(serial)),
          pending_frame(std::make_unique<LiteFrameSnapshot>()),
          working_frame(std::make_unique<LiteFrameSnapshot>()) {}

    int device_idx;
    uint16_t sensor_sn;
    std::string topic_prefix;
    std::string parameter_prefix;
    DeviceSettings settings;
    LiteOutputLayout output_layout;

    std::unique_ptr<LiteFrameSnapshot> pending_frame;
    std::unique_ptr<LiteFrameSnapshot> working_frame;
    iTFS::lite_vec3d_t reconstruction_map{};
    iTFS::packet::intrinsic_t intrinsic{};
    std::mutex frame_mutex;
    std::condition_variable frame_cv;
    std::thread worker_thread;
    bool frame_pending = false;
    bool worker_exit = false;
    bool calibration_ready = false;
    std::atomic<uint64_t> ros_frame_overwrite_count{0};
    std::atomic<bool> faulted{false};
    std::atomic<uint8_t> frame_request_mask{0};
    std::mutex status_mutex;
    LiteStatusSnapshot latest_status;
    bool status_received = false;
    bool info_published = false;
    std::chrono::steady_clock::time_point last_output_mode_warning_log_time{};

    sensor_msgs::msg::Image depth_msg;
    sensor_msgs::msg::Image amplitude_msg;
    sensor_msgs::msg::Image intensity_msg;
    sensor_msgs::msg::Image confidence_msg;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    bool camera_info_initialized = false;
    sensor_msgs::msg::PointCloud2 points_msg;
    bool points_schema_initialized = false;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr amplitude_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr intensity_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr confidence_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub;
};

// Normalize a configurable prefix once before appending fixed topic suffixes.
std::string topic_name(const std::string &prefix, const std::string &suffix) {
    std::string normalized = prefix.empty() ? "/ilidar_lite" : prefix;
    if (normalized.front() != '/') {
        normalized.insert(normalized.begin(), '/');
    }
    while (normalized.size() > 1 && normalized.back() == '/') {
        normalized.pop_back();
    }
    if (normalized == "/") {
        return suffix;
    }
    return normalized + suffix;
}

// Diagnostic KeyValue stores all values as strings; this helper keeps the
// conversion rules in one place.
template <typename T>
diagnostic_msgs::msg::KeyValue key_value(const std::string &key, const T &value) {
    diagnostic_msgs::msg::KeyValue kv;
    std::ostringstream ss;
    ss << value;
    kv.key = key;
    kv.value = ss.str();
    return kv;
}

std::string hex_string(uint32_t value) {
    std::ostringstream ss;
    ss << "0x" << std::hex << value;
    return ss.str();
}

bool parse_ipv4(const std::string &text, std::array<uint8_t, 4> &ip) {
    unsigned int a = 0;
    unsigned int b = 0;
    unsigned int c = 0;
    unsigned int d = 0;
    char tail = '\0';
    if (std::sscanf(text.c_str(), "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) != 4 ||
        a > 255 || b > 255 || c > 255 || d > 255) {
        return false;
    }
    ip = {static_cast<uint8_t>(a), static_cast<uint8_t>(b),
          static_cast<uint8_t>(c), static_cast<uint8_t>(d)};
    return true;
}

// Build one static transform from ROS roll/pitch/yaw parameters.
geometry_msgs::msg::TransformStamped make_transform(const rclcpp::Time &stamp,
                                                    const std::string &parent,
                                                    const std::string &child,
                                                    double tx,
                                                    double ty,
                                                    double tz,
                                                    double roll,
                                                    double pitch,
                                                    double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.transform.translation.x = tx;
    transform.transform.translation.y = ty;
    transform.transform.translation.z = tz;
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    return transform;
}

const char *depth_output_name(uint16_t mode) {
    switch (mode) {
    case iTFS::packet::info_v3_data_output_depth_off:
        return "OFF";
    case iTFS::packet::info_v3_data_output_depth_mm_16bit:
        return "DEPTH_MM_16";
    case iTFS::packet::info_v3_data_output_depth_raw_q16:
        return "DEPTH_RAW_Q16";
    case iTFS::packet::info_v3_data_output_depth_lin_8bit:
        return "DEPTH_LIN_8";
    case iTFS::packet::info_v3_data_output_depth_log_8bit:
        return "DEPTH_LOG_8";
    case iTFS::packet::info_v3_data_output_xyz_mm_16bit:
        return "XYZ_MM_16";
    case iTFS::packet::info_v3_data_output_xyz_raw_q15_q16:
        return "XYZ_RAW_Q15_Q16";
    case iTFS::packet::info_v3_data_output_xyz_lin_8bit:
        return "XYZ_LIN_8";
    default:
        return "UNKNOWN";
    }
}

const char *stream_output_name(uint16_t mode) {
    switch (mode) {
    case iTFS::packet::info_v3_data_output_stream_off:
        return "OFF";
    case iTFS::packet::info_v3_data_output_stream_raw_16bit:
        return "RAW_16";
    case iTFS::packet::info_v3_data_output_stream_lin_8bit:
        return "LIN_8";
    case iTFS::packet::info_v3_data_output_stream_log_8bit:
        return "LOG_8";
    default:
        return "UNKNOWN";
    }
}

const char *confidence_output_name(uint16_t mode) {
    switch (mode) {
    case iTFS::packet::info_v3_data_output_stream_off:
        return "OFF";
    case iTFS::packet::info_v3_data_output_stream_raw_16bit:
        return "RAW_16";
    case iTFS::packet::info_v3_data_output_confidence_mask_1bit_mode:
        return "MASK_1";
    default:
        return "UNKNOWN";
    }
}

LiteOutputLayout decode_output_layout(uint16_t data_output) {
    LiteOutputLayout layout;
    layout.data_output = data_output;
    layout.depth_mode = data_output & iTFS::packet::info_v3_data_output_depth_mask;
    layout.amplitude_mode =
        (data_output & iTFS::packet::info_v3_data_output_amplitude_mask) >>
        iTFS::packet::info_v3_data_output_amplitude_pos;
    layout.intensity_mode =
        (data_output & iTFS::packet::info_v3_data_output_intensity_mask) >>
        iTFS::packet::info_v3_data_output_intensity_pos;
    layout.confidence_mode =
        (data_output & iTFS::packet::info_v3_data_output_confidence_mask) >>
        iTFS::packet::info_v3_data_output_confidence_pos;
    layout.depth_8bit =
        layout.depth_mode == iTFS::packet::info_v3_data_output_depth_lin_8bit ||
        layout.depth_mode == iTFS::packet::info_v3_data_output_depth_log_8bit ||
        layout.depth_mode == iTFS::packet::info_v3_data_output_xyz_lin_8bit;
    layout.amplitude_8bit =
        layout.amplitude_mode == iTFS::packet::info_v3_data_output_stream_lin_8bit ||
        layout.amplitude_mode == iTFS::packet::info_v3_data_output_stream_log_8bit;
    layout.intensity_8bit =
        layout.intensity_mode == iTFS::packet::info_v3_data_output_stream_lin_8bit ||
        layout.intensity_mode == iTFS::packet::info_v3_data_output_stream_log_8bit;
    layout.confidence_mask1 =
        layout.confidence_mode == iTFS::packet::info_v3_data_output_confidence_mask_1bit_mode;
    layout.depth_name = depth_output_name(layout.depth_mode);
    layout.amplitude_name = stream_output_name(layout.amplitude_mode);
    layout.intensity_name = stream_output_name(layout.intensity_mode);
    layout.confidence_name = confidence_output_name(layout.confidence_mode);

    // These modes avoid ROS-side reconstruction and its quantization. XYZ_MM_16
    // is also native because its Z slot is already expressed in millimeters.
    std::vector<std::string> warnings;
    if (layout.depth_mode != iTFS::packet::info_v3_data_output_depth_off &&
        layout.depth_mode != iTFS::packet::info_v3_data_output_depth_mm_16bit &&
        layout.depth_mode != iTFS::packet::info_v3_data_output_xyz_mm_16bit) {
        const bool xyz_mode =
            layout.depth_mode == iTFS::packet::info_v3_data_output_xyz_raw_q15_q16 ||
            layout.depth_mode == iTFS::packet::info_v3_data_output_xyz_lin_8bit;
        warnings.push_back("depth=" + std::string(layout.depth_name) +
                           " (recommended " +
                           (xyz_mode ? std::string("XYZ_MM_16") : std::string("DEPTH_MM_16")) + ")");
    }
    if (layout.amplitude_mode != iTFS::packet::info_v3_data_output_stream_off &&
        layout.amplitude_mode != iTFS::packet::info_v3_data_output_stream_raw_16bit) {
        warnings.push_back("amplitude=" + std::string(layout.amplitude_name) +
                           " (recommended RAW_16)");
    }
    if (layout.intensity_mode != iTFS::packet::info_v3_data_output_stream_off &&
        layout.intensity_mode != iTFS::packet::info_v3_data_output_stream_raw_16bit) {
        warnings.push_back("intensity=" + std::string(layout.intensity_name) +
                           " (recommended RAW_16)");
    }
    if (layout.confidence_mode == iTFS::packet::info_v3_data_output_stream_raw_16bit) {
        warnings.push_back("confidence=RAW_16 (recommended MASK_1)");
    } else if (layout.confidence_mode != iTFS::packet::info_v3_data_output_stream_off &&
               layout.confidence_mode != iTFS::packet::info_v3_data_output_confidence_mask_1bit_mode) {
        warnings.push_back("confidence=" + std::string(layout.confidence_name) +
                           " (recommended MASK_1)");
    }

    std::ostringstream warning;
    for (size_t i = 0; i < warnings.size(); ++i) {
        if (i != 0) {
            warning << "; ";
        }
        warning << warnings[i];
    }
    layout.warning = warning.str();
    return layout;
}

const char *on_off_name(uint8_t bit) {
    return bit ? "ON" : "OFF";
}

uint16_t saturate_u16(float value) {
    if (value <= 0.0f) {
        return 0;
    }
    if (value >= 65535.0f) {
        return 65535;
    }
    return static_cast<uint16_t>(value + 0.5f);
}
} // namespace

class LiteCoreNode;
// The SDK exposes C-style callbacks without a user context pointer. This node
// is intentionally mono-instance, so an atomic bridge safely forwards callbacks
// while construction and destruction change the active instance.
static std::atomic<LiteCoreNode *> g_node{nullptr};

class LiteCoreNode : public rclcpp::Node {
  public:
    // Initialize all node-wide state before opening the UDP receiver. Per-SN
    // publishers and workers are deferred until the first valid info packet.
    LiteCoreNode() : Node("ilidar_lite_core") {
        for (auto &context : context_by_idx_) {
            context.store(nullptr, std::memory_order_relaxed);
        }
        for (auto &failed : context_init_failed_) {
            failed.store(false, std::memory_order_relaxed);
        }
        declare_parameters();

        // Fail before opening sockets if the copied SDK and packet definitions
        // are not from the same release.
        if (iTFS::lite_version() != 0) {
            throw std::runtime_error("iTFS-LITE API and packet versions do not match");
        }

        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(diagnostics_qos_depth_)).reliable());
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        diagnostics_timer_ = create_wall_timer(
            diagnostics_publish_period, [this] { publish_diagnostics(); });
        subscription_timer_ = create_wall_timer(
            subscription_poll_period, [this] { update_subscription_masks(); });

        uint8_t *broadcast_ptr = parse_optional_ip(broadcast_ip_, broadcast_ip_bytes_, "broadcast_ip");
        uint8_t *listening_ptr = parse_optional_ip(listening_ip_, listening_ip_bytes_, "listening_ip");

        // Publish the callback target before the SDK starts its receiver thread.
        g_node.store(this, std::memory_order_release);
        lite_ = std::make_unique<iTFS::LITE>(
            &LiteCoreNode::lidar_data_handler,
            &LiteCoreNode::status_packet_handler,
            &LiteCoreNode::info_packet_handler,
            broadcast_ptr,
            listening_ptr,
            static_cast<uint16_t>(listening_port_));

        // Match the standalone C++ examples: keep waiting until the SDK is ready.
        while (rclcpp::ok() && !lite_->Ready()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!rclcpp::ok()) {
            // Construction has not completed, so ~LiteCoreNode() will not run.
            // Stop the SDK receiver explicitly before leaving this constructor.
            g_node.store(nullptr, std::memory_order_release);
            lite_->Try_exit();
            lite_->Join();
            throw std::runtime_error("ROS shutdown while waiting for iTFS-LITE");
        }

        RCLCPP_INFO(get_logger(), "iTFS::LITE is ready. Waiting for up to %d devices.",
                    iTFS::max_device);
    }

    ~LiteCoreNode() override {
        // Stop forwarding first, then join the SDK receiver before destroying
        // buffers used by either callback or worker thread.
        g_node.store(nullptr, std::memory_order_release);

        if (lite_) {
            lite_->Try_exit();
            lite_->Join();
        }

        for (auto &owned_context : contexts_) {
            if (!owned_context) {
                continue;
            }
            {
                std::lock_guard<std::mutex> lk(owned_context->frame_mutex);
                owned_context->worker_exit = true;
                owned_context->frame_cv.notify_all();
            }
            if (owned_context->worker_thread.joinable()) {
                owned_context->worker_thread.join();
            }
        }
    }

  private:
    // Read-only descriptors make the parameter service match the node's
    // initialize-once policy instead of accepting unapplied runtime values.
    template <typename T>
    T declare_immutable_parameter(const std::string &name, const T &default_value) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        descriptor.description = "Fixed at node startup; restart to apply a new value";
        return declare_parameter<T>(name, default_value, descriptor);
    }

    // Declare all parameters before allocating publishers or opening the SDK.
    void declare_parameters() {
        broadcast_ip_ = declare_immutable_parameter<std::string>("broadcast_ip", "");
        listening_ip_ = declare_immutable_parameter<std::string>("listening_ip", "");
        listening_port_ = declare_immutable_parameter<int>("listening_port", iTFS::user_data_port);

        default_settings_.parent_frame_id =
            declare_immutable_parameter<std::string>("parent_frame_id", "base_link");
        default_settings_.publish_tf = declare_immutable_parameter<bool>("publish_tf", true);
        default_settings_.publish_depth = declare_immutable_parameter<bool>("publish_depth", true);
        default_settings_.publish_amplitude = declare_immutable_parameter<bool>("publish_amplitude", true);
        default_settings_.publish_intensity = declare_immutable_parameter<bool>("publish_intensity", true);
        default_settings_.publish_confidence = declare_immutable_parameter<bool>("publish_confidence", true);
        default_settings_.publish_camera_info = declare_immutable_parameter<bool>("publish_camera_info", true);
        default_settings_.publish_pointcloud = declare_immutable_parameter<bool>("publish_pointcloud", true);
        default_settings_.publish_pointcloud_amplitude =
            declare_immutable_parameter<bool>("publish_pointcloud_amplitude", true);
        default_settings_.publish_info = declare_immutable_parameter<bool>("publish_info", true);
        default_settings_.publish_diagnostics = declare_immutable_parameter<bool>("publish_diagnostics", true);
        default_settings_.sensor_qos_depth = static_cast<int>(
            std::max<int64_t>(declare_immutable_parameter<int64_t>("sensor_qos_depth", 5), 1));
        diagnostics_qos_depth_ = static_cast<int>(
            std::max<int64_t>(declare_immutable_parameter<int64_t>("diagnostics_qos_depth", 10), 1));

        // TODO(json@hybo.co): Replace the zero mount pose and optical
        // translation after the product offsets are measured. The optical
        // rotation is fixed by the ROS optical-frame axis convention.
        default_settings_.mount_translation_x = declare_immutable_parameter<double>("mount_translation_x", 0.0);
        default_settings_.mount_translation_y = declare_immutable_parameter<double>("mount_translation_y", 0.0);
        default_settings_.mount_translation_z = declare_immutable_parameter<double>("mount_translation_z", 0.0);
        default_settings_.mount_rotation_roll = declare_immutable_parameter<double>("mount_rotation_roll", 0.0);
        default_settings_.mount_rotation_pitch = declare_immutable_parameter<double>("mount_rotation_pitch", 0.0);
        default_settings_.mount_rotation_yaw = declare_immutable_parameter<double>("mount_rotation_yaw", 0.0);
        default_settings_.optical_translation_x = declare_immutable_parameter<double>("optical_translation_x", 0.0);
        default_settings_.optical_translation_y = declare_immutable_parameter<double>("optical_translation_y", 0.0);
        default_settings_.optical_translation_z = declare_immutable_parameter<double>("optical_translation_z", 0.0);
        default_settings_.optical_rotation_roll = declare_immutable_parameter<double>("optical_rotation_roll", -half_pi);
        default_settings_.optical_rotation_pitch = declare_immutable_parameter<double>("optical_rotation_pitch", 0.0);
        default_settings_.optical_rotation_yaw = declare_immutable_parameter<double>("optical_rotation_yaw", -half_pi);

        if (listening_port_ < 1 || listening_port_ > 65535) {
            throw std::invalid_argument("listening_port must be in the range 1..65535");
        }
    }

    // Advertise only products enabled for this SN. A sensor stream may still
    // remain silent when its immutable data_output mode is OFF.
    void create_publishers(DeviceContext &context) {
        rclcpp::SensorDataQoS sensor_qos;
        sensor_qos.keep_last(context.settings.sensor_qos_depth);

        if (context.settings.publish_depth) {
            context.depth_pub = create_publisher<sensor_msgs::msg::Image>(
                topic_name(context.topic_prefix, "/depth/image_raw"), sensor_qos);
        }
        if (context.settings.publish_camera_info) {
            context.depth_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(
                topic_name(context.topic_prefix, "/depth/camera_info"), sensor_qos);
        }
        if (context.settings.publish_amplitude) {
            context.amplitude_pub = create_publisher<sensor_msgs::msg::Image>(
                topic_name(context.topic_prefix, "/amplitude/image_raw"), sensor_qos);
        }
        if (context.settings.publish_intensity) {
            context.intensity_pub = create_publisher<sensor_msgs::msg::Image>(
                topic_name(context.topic_prefix, "/intensity/image_raw"), sensor_qos);
        }
        if (context.settings.publish_confidence) {
            context.confidence_pub = create_publisher<sensor_msgs::msg::Image>(
                topic_name(context.topic_prefix, "/confidence/image_raw"), sensor_qos);
        }
        if (context.settings.publish_pointcloud) {
            context.points_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
                topic_name(context.topic_prefix, "/points"), sensor_qos);
        }
        if (context.settings.publish_info) {
            rclcpp::QoS info_qos(rclcpp::KeepLast(1));
            info_qos.reliable();
            info_qos.transient_local();
            context.info_pub = create_publisher<std_msgs::msg::String>(
                topic_name(context.topic_prefix, "/info"), info_qos);
        }
    }

    template <typename T>
    T device_parameter(const DeviceContext &context,
                       const std::string &name,
                       const T &default_value) {
        return declare_immutable_parameter<T>(context.parameter_prefix + name, default_value);
    }

    DeviceSettings load_device_settings(DeviceContext &context) {
        DeviceSettings settings = default_settings_;
        const std::string identity = "ilidar_lite_" + std::to_string(context.sensor_sn);
        context.parameter_prefix = "devices." + identity + ".";

        settings.parent_frame_id = device_parameter(
            context, "parent_frame_id", default_settings_.parent_frame_id);
        settings.link_frame_id = device_parameter(
            context, "link_frame_id", identity + "_link");
        settings.optical_frame_id = device_parameter(
            context, "frame_id", identity + "_optical_frame");
        settings.publish_tf = device_parameter(context, "publish_tf", default_settings_.publish_tf);
        settings.publish_depth = device_parameter(
            context, "publish_depth", default_settings_.publish_depth);
        settings.publish_amplitude = device_parameter(
            context, "publish_amplitude", default_settings_.publish_amplitude);
        settings.publish_intensity = device_parameter(
            context, "publish_intensity", default_settings_.publish_intensity);
        settings.publish_confidence = device_parameter(
            context, "publish_confidence", default_settings_.publish_confidence);
        settings.publish_camera_info = device_parameter(
            context, "publish_camera_info", default_settings_.publish_camera_info);
        settings.publish_pointcloud = device_parameter(
            context, "publish_pointcloud", default_settings_.publish_pointcloud);
        settings.publish_pointcloud_amplitude = device_parameter(
            context, "publish_pointcloud_amplitude",
            default_settings_.publish_pointcloud_amplitude);
        settings.publish_info = device_parameter(
            context, "publish_info", default_settings_.publish_info);
        settings.publish_diagnostics = device_parameter(
            context, "publish_diagnostics", default_settings_.publish_diagnostics);
        settings.sensor_qos_depth = static_cast<int>(std::max<int64_t>(
            device_parameter<int64_t>(context, "sensor_qos_depth",
                                      default_settings_.sensor_qos_depth),
            1));

        settings.mount_translation_x = device_parameter(
            context, "mount_translation_x", default_settings_.mount_translation_x);
        settings.mount_translation_y = device_parameter(
            context, "mount_translation_y", default_settings_.mount_translation_y);
        settings.mount_translation_z = device_parameter(
            context, "mount_translation_z", default_settings_.mount_translation_z);
        settings.mount_rotation_roll = device_parameter(
            context, "mount_rotation_roll", default_settings_.mount_rotation_roll);
        settings.mount_rotation_pitch = device_parameter(
            context, "mount_rotation_pitch", default_settings_.mount_rotation_pitch);
        settings.mount_rotation_yaw = device_parameter(
            context, "mount_rotation_yaw", default_settings_.mount_rotation_yaw);
        settings.optical_translation_x = device_parameter(
            context, "optical_translation_x", default_settings_.optical_translation_x);
        settings.optical_translation_y = device_parameter(
            context, "optical_translation_y", default_settings_.optical_translation_y);
        settings.optical_translation_z = device_parameter(
            context, "optical_translation_z", default_settings_.optical_translation_z);
        settings.optical_rotation_roll = device_parameter(
            context, "optical_rotation_roll", default_settings_.optical_rotation_roll);
        settings.optical_rotation_pitch = device_parameter(
            context, "optical_rotation_pitch", default_settings_.optical_rotation_pitch);
        settings.optical_rotation_yaw = device_parameter(
            context, "optical_rotation_yaw", default_settings_.optical_rotation_yaw);
        return settings;
    }

    // Reject frame names that ROS TF cannot use consistently across tools.
    static void validate_frame_id(const std::string &frame_id, const char *parameter_name) {
        if (frame_id.empty()) {
            throw std::invalid_argument(std::string(parameter_name) + " must not be empty");
        }
        if (frame_id.front() == '/') {
            throw std::invalid_argument(std::string(parameter_name) +
                                        " must not start with '/'");
        }
        if (std::any_of(frame_id.begin(), frame_id.end(), [](unsigned char ch) {
                return std::isspace(ch) != 0;
            })) {
            throw std::invalid_argument(std::string(parameter_name) +
                                        " must not contain whitespace");
        }
    }

    // NaN/Inf transforms would poison the shared TF tree.
    static void validate_finite_transform_value(double value, const char *parameter_name) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument(std::string(parameter_name) + " must be finite");
        }
    }

    // Child frame IDs must be unique across sensors. Shared parent frames are
    // valid and expected; invalid numeric values or equal local frames would
    // otherwise publish unusable transforms.
    void validate_device_frames(const DeviceContext &candidate) const {
        const auto &settings = candidate.settings;
        validate_frame_id(settings.parent_frame_id, "parent_frame_id");
        validate_frame_id(settings.link_frame_id, "link_frame_id");
        validate_frame_id(settings.optical_frame_id, "frame_id");
        validate_finite_transform_value(settings.mount_translation_x, "mount_translation_x");
        validate_finite_transform_value(settings.mount_translation_y, "mount_translation_y");
        validate_finite_transform_value(settings.mount_translation_z, "mount_translation_z");
        validate_finite_transform_value(settings.mount_rotation_roll, "mount_rotation_roll");
        validate_finite_transform_value(settings.mount_rotation_pitch, "mount_rotation_pitch");
        validate_finite_transform_value(settings.mount_rotation_yaw, "mount_rotation_yaw");
        validate_finite_transform_value(settings.optical_translation_x, "optical_translation_x");
        validate_finite_transform_value(settings.optical_translation_y, "optical_translation_y");
        validate_finite_transform_value(settings.optical_translation_z, "optical_translation_z");
        validate_finite_transform_value(settings.optical_rotation_roll, "optical_rotation_roll");
        validate_finite_transform_value(settings.optical_rotation_pitch, "optical_rotation_pitch");
        validate_finite_transform_value(settings.optical_rotation_yaw, "optical_rotation_yaw");
        if (settings.parent_frame_id == settings.link_frame_id ||
            settings.parent_frame_id == settings.optical_frame_id ||
            settings.link_frame_id == settings.optical_frame_id) {
            throw std::invalid_argument("parent_frame_id, link_frame_id, and frame_id must be distinct");
        }
        for (const auto &other : contexts_) {
            if (!other) {
                continue;
            }
            const bool duplicate_child =
                settings.link_frame_id == other->settings.link_frame_id ||
                settings.link_frame_id == other->settings.optical_frame_id ||
                settings.optical_frame_id == other->settings.link_frame_id ||
                settings.optical_frame_id == other->settings.optical_frame_id;
            if (duplicate_child) {
                throw std::invalid_argument("TF child frame IDs collide with SN#" +
                                            std::to_string(other->sensor_sn));
            }
        }
    }

    // SDK callbacks use this lock-free lookup after initialization publishes
    // the fully constructed context with release ordering.
    DeviceContext *context_for(int device_idx) const {
        if (device_idx < 0 || device_idx >= iTFS::max_device) {
            return nullptr;
        }
        return context_by_idx_[device_idx].load(std::memory_order_acquire);
    }

    // Create exactly one context for a valid device index/SN pair. The mutex
    // serializes first discovery while later callbacks take the atomic path.
    DeviceContext *initialize_context(iTFS::lite_device_t *device) {
        if (device->idx < 0 || device->idx >= iTFS::max_device ||
            device->info_v3.sensor_sn == 0xFFFF) {
            return nullptr;
        }
        if (context_init_failed_[device->idx].load(std::memory_order_relaxed)) {
            return nullptr;
        }
        if (DeviceContext *existing = context_for(device->idx)) {
            return existing;
        }

        std::lock_guard<std::mutex> lk(context_init_mutex_);
        if (DeviceContext *existing = context_for(device->idx)) {
            return existing;
        }
        for (const auto &other : contexts_) {
            if (other && other->sensor_sn == device->info_v3.sensor_sn) {
                RCLCPP_ERROR(get_logger(), "Duplicate sensor SN#%u on D#%d and D#%d.",
                             device->info_v3.sensor_sn, other->device_idx, device->idx);
                context_init_failed_[device->idx].store(true, std::memory_order_relaxed);
                return nullptr;
            }
        }

        try {
            auto owned = std::make_unique<DeviceContext>(device->idx, device->info_v3.sensor_sn);
            owned->settings = load_device_settings(*owned);
            validate_device_frames(*owned);
            owned->output_layout = decode_output_layout(device->info_v3.data_output);
            create_publishers(*owned);
            if (owned->settings.publish_tf) {
                publish_static_tf(*owned);
            }

            DeviceContext *context = owned.get();
            context->worker_thread = std::thread(&LiteCoreNode::frame_worker, this, context);
            contexts_[device->idx] = std::move(owned);
            context_by_idx_[device->idx].store(context, std::memory_order_release);
            RCLCPP_INFO(get_logger(), "D#%d SN#%u initialized at %s.",
                        device->idx, context->sensor_sn, context->topic_prefix.c_str());
            return context;
        } catch (const std::exception &exc) {
            context_init_failed_[device->idx].store(true, std::memory_order_relaxed);
            RCLCPP_ERROR(get_logger(), "D#%d SN#%u initialization failed: %s",
                         device->idx, device->info_v3.sensor_sn, exc.what());
            return nullptr;
        }
    }

    // The SDK accepts nullable four-byte IPv4 arrays; an empty ROS parameter
    // deliberately selects the SDK's automatic interface discovery.
    uint8_t *parse_optional_ip(const std::string &text,
                               std::array<uint8_t, 4> &storage,
                               const char *parameter_name) {
        if (text.empty()) {
            return nullptr;
        }
        if (!parse_ipv4(text, storage)) {
            throw std::invalid_argument(std::string(parameter_name) + " is not a valid IPv4 address");
        }
        return storage.data();
    }

    // Image callback: forward the SDK-owned frame for active-slot copying.
    static void lidar_data_handler(iTFS::lite_device_t *device) {
        if (LiteCoreNode *node = g_node.load(std::memory_order_acquire)) {
            node->handle_lidar_data(device);
        }
    }

    // Status callback: inspect status_v3 immediately; never retain device.
    static void status_packet_handler(iTFS::lite_device_t *device) {
        if (LiteCoreNode *node = g_node.load(std::memory_order_acquire)) {
            node->handle_status_packet(device);
        }
    }

    // Info callback: initialize the immutable output layout from info_v3.
    static void info_packet_handler(iTFS::lite_device_t *device) {
        if (LiteCoreNode *node = g_node.load(std::memory_order_acquire)) {
            node->handle_info_packet(device);
        }
    }

    // Called only by the subscription timer, never by an SDK callback.
    FrameRequest sample_frame_request(const DeviceContext &context) const {
        FrameRequest request;
        request.depth = has_subscribers(context.depth_pub);
        request.amplitude = has_subscribers(context.amplitude_pub);
        request.intensity = has_subscribers(context.intensity_pub);
        request.confidence = has_subscribers(context.confidence_pub);
        request.camera_info = has_subscribers(context.depth_camera_info_pub);
        request.pointcloud = has_subscribers(context.points_pub);
        return request;
    }

    // DDS graph queries stay on the ROS executor. The SDK callback consumes
    // only this cached byte, avoiding middleware locks in the receive thread.
    void update_subscription_masks() {
        for (int device_idx = 0; device_idx < iTFS::max_device; ++device_idx) {
            if (DeviceContext *context = context_for(device_idx)) {
                context->frame_request_mask.store(
                    sample_frame_request(*context).mask(), std::memory_order_release);
            }
        }
    }

    // Return the active packed-slot width. Eight-bit modes copy half the bytes
    // of a 16-bit slot; X/Y inherit the selected XYZ depth width.
    size_t image_class_bytes(const LiteOutputLayout &layout, uint8_t img_class) const {
        bool image_8bit = false;
        switch (img_class) {
        case iTFS::lite_img_depth:
        case iTFS::lite_img_point_x:
        case iTFS::lite_img_point_y:
            image_8bit = layout.depth_8bit;
            break;
        case iTFS::lite_img_amplitude:
            image_8bit = layout.amplitude_8bit;
            break;
        case iTFS::lite_img_intensity:
            image_8bit = layout.intensity_8bit;
            break;
        case iTFS::lite_img_confidence:
            image_8bit = layout.confidence_mask1;
            break;
        default:
            return 0;
        }
        return static_cast<size_t>(iTFS::lite_max_row) * iTFS::lite_max_col *
               (image_8bit ? sizeof(uint8_t) : sizeof(uint16_t));
    }

    // Repack only the classes needed by current subscribers. Offsets are
    // rewritten for the compact destination, so the unmodified SDK getters can
    // resolve it exactly like the OpenCV example's full snapshot.
    void copy_requested_image_classes(const DeviceContext &context,
                                      const FrameRequest &request,
                                      const iTFS::lite_img_t &source,
                                      iTFS::lite_img_cpy_t &destination) const {
        iTFS::copy_img_metadata(&destination,
                                const_cast<iTFS::lite_img_t *>(&source));
        std::fill(std::begin(destination.img_offset),
                  std::end(destination.img_offset), -1);
        std::fill(std::begin(destination.data_class),
                  std::end(destination.data_class), 0);
        destination.data_class_count = 0;

        std::array<bool, iTFS::lite_img_class_count> copy_class{};
        copy_class[iTFS::lite_img_depth] = request.depth || request.pointcloud;
        copy_class[iTFS::lite_img_amplitude] = request.amplitude ||
            (request.pointcloud && context.settings.publish_pointcloud_amplitude);
        copy_class[iTFS::lite_img_intensity] = request.intensity;
        copy_class[iTFS::lite_img_confidence] = request.confidence;
        if (request.pointcloud && source.xyz_on) {
            copy_class[iTFS::lite_img_point_x] = true;
            copy_class[iTFS::lite_img_point_y] = true;
        }

        for (uint8_t img_class = 0; img_class < iTFS::lite_img_class_count; ++img_class) {
            if (!copy_class[img_class] || source.img_offset[img_class] < 0) {
                continue;
            }
            const int source_slot = source.img_offset[img_class] / iTFS::lite_max_row;
            if (source_slot < 0 || source_slot >= source.data_class_count) {
                continue;
            }
            const uint8_t destination_slot = destination.data_class_count++;
            destination.data_class[destination_slot] = img_class;
            destination.img_offset[img_class] = destination_slot * iTFS::lite_max_row;
            std::memcpy(&destination.data[destination_slot],
                        &source.data[source_slot],
                        image_class_bytes(context.output_layout, img_class));
        }
    }

    // Copy only requested packed slots while the SDK-owned frame is stable.
    // There is one pending slot per device, isolating slow consumers.
    void handle_lidar_data(iTFS::lite_device_t *device) {
        DeviceContext *context = context_for(device->idx);
        if (context == nullptr || context->faulted.load(std::memory_order_relaxed)) {
            return;
        }
        const FrameRequest request = FrameRequest::from_mask(
            context->frame_request_mask.load(std::memory_order_acquire));
        if (!request.any()) {
            return;
        }

        std::lock_guard<std::mutex> lk(context->frame_mutex);
        // Count only ROS-side loss: the SDK completed both frames, but the
        // worker had not claimed the older pending snapshot before replacement.
        if (context->frame_pending) {
            context->ros_frame_overwrite_count.fetch_add(1, std::memory_order_relaxed);
        }
        copy_requested_image_classes(*context, request, device->data,
                                     context->pending_frame->lite_img_data);
        context->pending_frame->stamp = now();
        context->pending_frame->request_mask = request.mask();
        // Calibration becomes immutable after the first valid SDK map, so the
        // device worker can read its map without holding frame_mutex.
        if ((request.camera_info || request.pointcloud) &&
            !context->calibration_ready && device->reconstruction_map.valid) {
            context->reconstruction_map = device->reconstruction_map;
            context->intrinsic = device->intrinsic;
            context->calibration_ready = true;
            RCLCPP_INFO(get_logger(), "D#%d intrinsic and reconstruction map are ready.", device->idx);
        }
        context->pending_frame->calibration_ready = context->calibration_ready;
        context->frame_pending = true;
        context->frame_cv.notify_one();
    }

    // Keep the receive callback bounded to fixed-size copies and counters.
    // Interval OR-latches preserve one-frame faults until the next publication.
    void handle_status_packet(iTFS::lite_device_t *device) {
        DeviceContext *context = context_for(device->idx);
        if (context == nullptr || !context->settings.publish_diagnostics || !diagnostics_pub_) {
            return;
        }
        std::lock_guard<std::mutex> lk(context->status_mutex);
        context->latest_status.status = device->status_v3;
        context->latest_status.image_frame_status = device->data.frame_status;
        context->latest_status.pending_sensor_warning |= device->status_v3.sensor_warning;
        context->latest_status.pending_sensor_frame_status |=
            device->status_v3.sensor_frame_status;
        context->latest_status.pending_image_frame_status |= device->data.frame_status;
        ++context->latest_status.status_packet_count;
        if (device->status_v3.sensor_warning != 0) {
            context->latest_status.sensor_warning_event = {
                device->status_v3, device->data.frame_status, true};
            ++context->latest_status.sensor_warning_frame_count;
        }
        if (device->status_v3.sensor_frame_status != 0) {
            context->latest_status.sensor_frame_warning_event = {
                device->status_v3, device->data.frame_status, true};
            ++context->latest_status.sensor_frame_warning_frame_count;
        }
        if (device->data.frame_status != 0) {
            context->latest_status.image_frame_error_event = {
                device->status_v3, device->data.frame_status, true};
            ++context->latest_status.image_frame_error_frame_count;
        }
        context->status_received = true;
    }

    // Publish one aggregated array at a fixed low rate. This keeps allocations,
    // string formatting, logging, and DDS work out of the SDK receiver thread.
    void publish_diagnostics() {
        if (!diagnostics_pub_) {
            return;
        }
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = now();
        array.status.reserve(iTFS::max_device);

        for (int device_idx = 0; device_idx < iTFS::max_device; ++device_idx) {
            DeviceContext *context = context_for(device_idx);
            if (context == nullptr || !context->settings.publish_diagnostics) {
                continue;
            }

            LiteStatusSnapshot snapshot;
            {
                std::lock_guard<std::mutex> lk(context->status_mutex);
                if (!context->status_received) {
                    continue;
                }
                snapshot = context->latest_status;
                // The copied snapshot owns every event since the previous tick;
                // only interval latches reset, while cumulative counts remain.
                context->latest_status.pending_sensor_warning = 0;
                context->latest_status.pending_sensor_frame_status = 0;
                context->latest_status.pending_image_frame_status = 0;
                context->latest_status.sensor_warning_event.valid = false;
                context->latest_status.sensor_frame_warning_event.valid = false;
                context->latest_status.image_frame_error_event.valid = false;
            }

            update_output_mode_log(*context);
            const auto &sensor = snapshot.status;
            const uint64_t time_us = iTFS::packet::get_sensor_time_in_us(
                &snapshot.status);
            const uint32_t reported_sensor_warning =
                sensor.sensor_warning | snapshot.pending_sensor_warning;
            const uint16_t reported_sensor_frame_status =
                sensor.sensor_frame_status | snapshot.pending_sensor_frame_status;
            const int reported_image_frame_status =
                snapshot.image_frame_status | snapshot.pending_image_frame_status;
            const bool image_frame_dropped = reported_image_frame_status != 0;
            const bool sensor_frame_warning = reported_sensor_frame_status != 0;
            const bool context_faulted = context->faulted.load(std::memory_order_relaxed);

            // Select the same priority used by status.message. Pending events
            // carry their original frame/time; a still-active latest fault uses
            // the latest status directly after its interval event was drained.
            iTFS::packet::status_v3_t *event_status = nullptr;
            int event_image_frame_status = 0;
            const char *event_type = nullptr;
            if (snapshot.sensor_warning_event.valid) {
                event_status = &snapshot.sensor_warning_event.status;
                event_image_frame_status = snapshot.sensor_warning_event.image_frame_status;
                event_type = "sensor_warning";
            } else if (sensor.sensor_warning != 0) {
                event_status = &snapshot.status;
                event_image_frame_status = snapshot.image_frame_status;
                event_type = "sensor_warning";
            } else if (snapshot.sensor_frame_warning_event.valid) {
                event_status = &snapshot.sensor_frame_warning_event.status;
                event_image_frame_status = snapshot.sensor_frame_warning_event.image_frame_status;
                event_type = "sensor_frame_warning";
            } else if (sensor.sensor_frame_status != 0) {
                event_status = &snapshot.status;
                event_image_frame_status = snapshot.image_frame_status;
                event_type = "sensor_frame_warning";
            } else if (snapshot.image_frame_error_event.valid) {
                event_status = &snapshot.image_frame_error_event.status;
                event_image_frame_status = snapshot.image_frame_error_event.image_frame_status;
                event_type = "image_frame_error";
            } else if (snapshot.image_frame_status != 0) {
                event_status = &snapshot.status;
                event_image_frame_status = snapshot.image_frame_status;
                event_type = "image_frame_error";
            }

            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name = "ilidar_lite_" + std::to_string(context->sensor_sn) + "/status";
            status.hardware_id = std::to_string(sensor.sensor_sn);
            status.level = context_faulted ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
                           (reported_sensor_warning == 0 && !sensor_frame_warning &&
                                    !image_frame_dropped &&
                                    !context->output_layout.conversion_required() ?
                                diagnostic_msgs::msg::DiagnosticStatus::OK :
                                diagnostic_msgs::msg::DiagnosticStatus::WARN);
            status.message = context_faulted ? "DATA_OUTPUT changed; restart required" :
                             (reported_sensor_warning != 0 ? "sensor warning" :
                              (sensor_frame_warning ? "sensor frame warning" :
                               (image_frame_dropped ? "image frame has missing rows" :
                                (context->output_layout.conversion_required() ?
                                     "non-native image encoding" : "OK"))));
            status.values = {
                key_value("ros_stamp_sec", array.header.stamp.sec),
                key_value("ros_stamp_nanosec", array.header.stamp.nanosec),
                key_value("sensor_time_us", time_us),
                key_value("device_index", context->device_idx),
                key_value("capture_mode", static_cast<unsigned int>(sensor.capture_mode)),
                key_value("capture_frame", static_cast<unsigned int>(sensor.capture_frame)),
                key_value("sensor_frame_status", hex_string(reported_sensor_frame_status)),
                key_value("image_frame_status", reported_image_frame_status),
                key_value("temperature_rx_c", static_cast<float>(sensor.sensor_temp_rx) * 0.01f),
                key_value("temperature_tx_c", static_cast<float>(sensor.sensor_temp_tx) * 0.01f),
                key_value("temperature_core_c", static_cast<float>(sensor.sensor_temp_core) * 0.01f),
                key_value("exposure_us", sensor.sensor_exposure),
                key_value("usb_voltage_v", static_cast<float>(sensor.sensor_usb_level) * 0.01f),
                key_value("power0_voltage_v", static_cast<float>(sensor.sensor_power_level[0]) * 0.01f),
                key_value("power1_voltage_v", static_cast<float>(sensor.sensor_power_level[1]) * 0.01f),
                key_value("cpu_usage_percent", static_cast<float>(sensor.sensor_cpu_usage) * 0.01f),
                key_value("ram_usage_percent", static_cast<float>(sensor.sensor_ram_usage) * 0.01f),
                key_value("frame_drop_count", sensor.sensor_frame_drop_count),
                key_value("status_packet_count", snapshot.status_packet_count),
                key_value("sensor_warning_frame_count",
                          snapshot.sensor_warning_frame_count),
                key_value("sensor_frame_warning_frame_count",
                          snapshot.sensor_frame_warning_frame_count),
                key_value("image_frame_error_frame_count",
                          snapshot.image_frame_error_frame_count),
                key_value("ros_frame_overwrite_count",
                          context->ros_frame_overwrite_count.load(std::memory_order_relaxed)),
                key_value("udp_rx_drop_count", sensor.sensor_udp_rx_drop_count),
                key_value("udp_tx_drop_count", sensor.sensor_udp_tx_drop_count),
                key_value("warning", hex_string(reported_sensor_warning)),
                key_value("topic_prefix", context->topic_prefix),
                key_value("parent_frame_id", context->settings.parent_frame_id),
                key_value("link_frame_id", context->settings.link_frame_id),
                key_value("optical_frame_id", context->settings.optical_frame_id),
                key_value("depth_input_mode", context->output_layout.depth_name),
                key_value("amplitude_input_mode", context->output_layout.amplitude_name),
                key_value("intensity_input_mode", context->output_layout.intensity_name),
                key_value("confidence_input_mode", context->output_layout.confidence_name),
                key_value("image_conversion_required",
                          context->output_layout.conversion_required() ? "true" : "false"),
                key_value("image_conversion_warning", context->output_layout.warning)};
            if (event_status != nullptr) {
                const uint64_t event_time_us = iTFS::packet::get_sensor_time_in_us(
                    event_status);
                status.values.push_back(key_value("event_type", event_type));
                status.values.push_back(key_value("event_sensor_time_us", event_time_us));
                status.values.push_back(key_value(
                    "event_capture_frame",
                    static_cast<unsigned int>(event_status->capture_frame)));
                status.values.push_back(key_value(
                    "event_sensor_warning", hex_string(event_status->sensor_warning)));
                status.values.push_back(key_value(
                    "event_sensor_frame_status",
                    hex_string(event_status->sensor_frame_status)));
                status.values.push_back(key_value(
                    "event_image_frame_status", event_image_frame_status));
            }
            array.status.push_back(std::move(status));
        }

        if (!array.status.empty()) {
            diagnostics_pub_->publish(array);
        }
    }

    // Cache the immutable conversion layout and publish one latched summary.
    // Later packets are used only to reject an unsafe DATA_OUTPUT change.
    void handle_info_packet(iTFS::lite_device_t *device) {
        DeviceContext *context = initialize_context(device);
        if (context == nullptr) {
            return;
        }

        const uint16_t data_output = device->info_v3.data_output;
        if (device->info_v3.sensor_sn != context->sensor_sn) {
            RCLCPP_ERROR(get_logger(), "D#%d changed SN from %u to %u.", device->idx,
                         context->sensor_sn, device->info_v3.sensor_sn);
            context->faulted.store(true, std::memory_order_relaxed);
            return;
        }
        if (data_output != context->output_layout.data_output) {
            RCLCPP_ERROR(get_logger(),
                         "D#%d DATA_OUTPUT changed from %s to %s while running. "
                         "Publishing for SN#%u is disabled until restart.",
                         device->idx,
                         hex_string(context->output_layout.data_output).c_str(),
                         hex_string(data_output).c_str(), context->sensor_sn);
            context->faulted.store(true, std::memory_order_relaxed);
            return;
        }
        if (!context->settings.publish_info || !context->info_pub || context->info_published) {
            return;
        }
        std::ostringstream ss;
        ss << "SN# " << device->info_v3.sensor_sn
           << " D# " << device->idx
           << " LOCK " << on_off_name(device->info_v3.lock)
           << " CAPTURE_MODE " << static_cast<unsigned int>(device->info_v3.capture_mode)
           << " DATA_OUTPUT " << hex_string(data_output)
           << " TOPIC " << context->topic_prefix
           << " DEPTH " << context->output_layout.depth_name
           << " AMPLITUDE " << context->output_layout.amplitude_name
           << " INTENSITY " << context->output_layout.intensity_name
           << " CONFIDENCE " << context->output_layout.confidence_name
           << " PERIOD " << device->info_v3.capture_period_ns << " ns"
           << " SENSOR_IP "
           << static_cast<unsigned int>(device->info_v3.data_sensor_ip[0]) << "."
           << static_cast<unsigned int>(device->info_v3.data_sensor_ip[1]) << "."
           << static_cast<unsigned int>(device->info_v3.data_sensor_ip[2]) << "."
           << static_cast<unsigned int>(device->info_v3.data_sensor_ip[3])
           << " DEST "
           << static_cast<unsigned int>(device->info_v3.data_dest_ip[0]) << "."
           << static_cast<unsigned int>(device->info_v3.data_dest_ip[1]) << "."
           << static_cast<unsigned int>(device->info_v3.data_dest_ip[2]) << "."
           << static_cast<unsigned int>(device->info_v3.data_dest_ip[3]) << ":"
           << device->info_v3.data_port
           << " FW V" << static_cast<unsigned int>(device->info_v3.sensor_fw_ver[2])
           << "." << static_cast<unsigned int>(device->info_v3.sensor_fw_ver[1])
           << "." << static_cast<unsigned int>(device->info_v3.sensor_fw_ver[0]);

        std_msgs::msg::String msg;
        msg.data = ss.str();
        context->info_pub->publish(msg);
        context->info_published = true;
        RCLCPP_INFO(get_logger(), "%s", msg.data.c_str());
    }

    // Console output repeats at a bounded rate because the mode set is
    // immutable; diagnostics still carry the warning on every timer tick.
    void update_output_mode_log(DeviceContext &context) {
        const auto now_steady = std::chrono::steady_clock::now();
        if (context.output_layout.warning.empty()) {
            return;
        }

        const bool repeat_due = context.last_output_mode_warning_log_time.time_since_epoch().count() == 0 ||
                                now_steady - context.last_output_mode_warning_log_time >=
                                    output_mode_warning_repeat_period;
        if (repeat_due) {
            RCLCPP_WARN(get_logger(), "D#%d SN#%u non-native ROS image modes: %s",
                        context.device_idx, context.sensor_sn,
                        context.output_layout.warning.c_str());
            context.last_output_mode_warning_log_time = now_steady;
        }
    }

    // Swap buffers under the mutex, then perform all expensive ROS conversion
    // outside the SDK callback path. The callback remains free to replace the
    // next pending frame while this worker publishes the current one.
    void frame_worker(DeviceContext *context) {
        while (true) {
            {
                std::unique_lock<std::mutex> lk(context->frame_mutex);
                context->frame_cv.wait(lk, [context] {
                    return context->frame_pending || context->worker_exit;
                });
                if (context->worker_exit) {
                    return;
                }
                context->pending_frame.swap(context->working_frame);
                context->frame_pending = false;
            }
            publish_frame(*context, *context->working_frame);
        }
    }

    // Subscription counts are queried only by the low-rate ROS timer. A newly
    // joined subscriber begins receiving after at most one polling interval
    // plus one sensor capture period.
    template <typename PublisherSharedPtr>
    bool has_subscribers(const PublisherSharedPtr &publisher) const {
        return publisher &&
               (publisher->get_subscription_count() > 0 ||
                publisher->get_intra_process_subscription_count() > 0);
    }

    // Generate every requested ROS product from the same snapshot, request mask,
    // and timestamp. Disabled data_output streams have no valid SDK slot.
    void publish_frame(DeviceContext &context, const LiteFrameSnapshot &frame) {
        const FrameRequest request = FrameRequest::from_mask(frame.request_mask);
        const bool has_depth_output = frame.lite_img_data.depth_on || frame.lite_img_data.xyz_on;
        const bool pointcloud_valid = request.pointcloud &&
                                      has_depth_output &&
                                      (frame.lite_img_data.xyz_on || frame.calibration_ready);

        const bool amplitude_needed = request.amplitude ||
                                      (pointcloud_valid &&
                                       context.settings.publish_pointcloud_amplitude);

        // Match lite_opencv_example.cpp's user-processing order. Each block
        // resolves the SDK slot, restores the public ROS unit, then publishes.
        process_depth_image(context, frame, request.depth && has_depth_output);
        const bool amplitude_valid = process_amplitude_image(
            context, frame, amplitude_needed, request.amplitude);
        process_intensity_image(context, frame, request.intensity);
        process_confidence_image(context, frame, request.confidence);

        if (request.camera_info && has_depth_output && frame.calibration_ready) {
            publish_camera_info(context, frame);
        }

        if (pointcloud_valid) {
            publish_pointcloud(context, frame,
                               amplitude_valid ? &context.amplitude_msg : nullptr);
        }
    }

    // ROS image storage is declared little-endian regardless of host alignment.
    static void write_u16_le(uint8_t *destination, uint16_t value) {
        destination[0] = static_cast<uint8_t>(value & 0xFFu);
        destination[1] = static_cast<uint8_t>(value >> 8);
    }

    static uint16_t read_u16_le(const uint8_t *source) {
        return static_cast<uint16_t>(source[0]) |
               (static_cast<uint16_t>(source[1]) << 8);
    }

    // Native 16-bit sensor modes already match ROS mono16 on supported
    // little-endian targets. Big-endian hosts fall back to the conversion loop.
    static bool copy_native_u16_image(const uint16_t *source,
                                      std::vector<uint8_t> &destination) {
        const uint16_t endian_probe = 1;
        const bool little_endian =
            *reinterpret_cast<const uint8_t *>(&endian_probe) == 1;
        const size_t image_bytes = static_cast<size_t>(iTFS::lite_max_row) *
                                   iTFS::lite_max_col * sizeof(uint16_t);
        if (!little_endian || source == nullptr || destination.size() < image_bytes) {
            return false;
        }
        std::memcpy(destination.data(), source, image_bytes);
        return true;
    }

    // Reuse each message buffer and reset only its fixed 320x240 contract.
    // resize() retains capacity after the first published frame.
    void prepare_image_message(sensor_msgs::msg::Image &msg,
                               const std::string &frame_id,
                               const char *encoding,
                               size_t bytes_per_pixel) const {
        msg.header.frame_id = frame_id;
        msg.height = iTFS::lite_max_row;
        msg.width = iTFS::lite_max_col;
        msg.encoding = encoding;
        msg.is_bigendian = false;
        msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(
            iTFS::lite_max_col * bytes_per_pixel);
        msg.data.resize(static_cast<size_t>(iTFS::lite_max_row) * msg.step);
    }

    // Apply the snapshot timestamp only after conversion has completed.
    void publish_image(
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher,
        sensor_msgs::msg::Image &msg,
        const rclcpp::Time &stamp) const {
        if (!publisher) {
            return;
        }
        msg.header.stamp = stamp;
        publisher->publish(msg);
    }

    // Product processors keep subscription checks outside the pixel loops and
    // publish only after a complete conversion succeeds.
    void process_depth_image(DeviceContext &context,
                             const LiteFrameSnapshot &frame,
                             bool requested) {
        if (!requested) {
            return;
        }
        prepare_image_message(context.depth_msg, context.settings.optical_frame_id,
                              "mono16", sizeof(uint16_t));
        if (restore_depth_mm(context, frame, context.depth_msg.data)) {
            publish_image(context.depth_pub, context.depth_msg, frame.stamp);
        }
    }

    // Amplitude restoration also supplies the optional point-cloud field.
    bool process_amplitude_image(DeviceContext &context,
                                 const LiteFrameSnapshot &frame,
                                 bool needed,
                                 bool publish_requested) {
        if (!needed) {
            return false;
        }
        prepare_image_message(context.amplitude_msg, context.settings.optical_frame_id,
                              "mono16", sizeof(uint16_t));
        const bool valid = restore_amplitude_raw(
            context, frame, context.amplitude_msg.data);
        if (valid && publish_requested) {
            publish_image(context.amplitude_pub, context.amplitude_msg, frame.stamp);
        }
        return valid;
    }

    // Intensity has the same public mono16 contract but is not used by points.
    void process_intensity_image(DeviceContext &context,
                                 const LiteFrameSnapshot &frame,
                                 bool requested) {
        if (!requested || !frame.lite_img_data.intensity_on) {
            return;
        }
        prepare_image_message(context.intensity_msg, context.settings.optical_frame_id,
                              "mono16", sizeof(uint16_t));
        if (restore_intensity_raw(context, frame, context.intensity_msg.data)) {
            publish_image(context.intensity_pub, context.intensity_msg, frame.stamp);
        }
    }

    // Confidence is normalized to a binary mono8 validity mask.
    void process_confidence_image(DeviceContext &context,
                                  const LiteFrameSnapshot &frame,
                                  bool requested) {
        if (!requested || !frame.lite_img_data.confidence_on) {
            return;
        }
        prepare_image_message(context.confidence_msg, context.settings.optical_frame_id,
                              "mono8", sizeof(uint8_t));
        if (restore_confidence_mask(context, frame, context.confidence_msg.data)) {
            publish_image(context.confidence_pub, context.confidence_msg, frame.stamp);
        }
    }

    // Normalize depth or native XYZ-Z into the public ROS contract: unsigned
    // 16-bit millimeters. Invalid zero values remain zero.
    bool restore_depth_mm(const DeviceContext &context,
                          const LiteFrameSnapshot &frame,
                          std::vector<uint8_t> &depth_mm) const {
        // SDK getters are not const-qualified even though they only resolve a
        // packed slot here; const_cast is limited to these access calls.
        const uint8_t (*depth_image8)[iTFS::lite_max_col] = context.output_layout.depth_8bit ?
            iTFS::get_img_u8_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data), iTFS::lite_img_depth) : nullptr;
        const uint16_t (*depth_image16)[iTFS::lite_max_col] = !context.output_layout.depth_8bit ?
            iTFS::get_img_u16_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data), iTFS::lite_img_depth) : nullptr;
        if (depth_image8 == nullptr && depth_image16 == nullptr) {
            return false;
        }

        const bool native_mm =
            context.output_layout.depth_mode ==
                iTFS::packet::info_v3_data_output_depth_mm_16bit ||
            context.output_layout.depth_mode ==
                iTFS::packet::info_v3_data_output_xyz_mm_16bit;
        if (native_mm && depth_image16 != nullptr && copy_native_u16_image(
                             &depth_image16[0][0], depth_mm)) {
            return true;
        }

        for (int r = 0; r < iTFS::lite_max_row; ++r) {
            for (int c = 0; c < iTFS::lite_max_col; ++c) {
                float value_mm = 0.0f;
                if (context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_depth_mm_16bit ||
                    context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_xyz_mm_16bit) {
                    value_mm = static_cast<float>(depth_image16[r][c]);
                } else if (context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_depth_raw_q16 ||
                           context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_xyz_raw_q15_q16) {
                    value_mm = static_cast<float>(depth_image16[r][c]) * depth_raw_q16_to_m * 1000.0f;
                } else if (context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_depth_lin_8bit ||
                           context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_xyz_lin_8bit) {
                    value_mm = static_cast<float>(depth_image8[r][c]) * depth_lin8_to_m * 1000.0f;
                } else if (context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_depth_log_8bit) {
                    value_mm = static_cast<float>(iTFS::depth_log8_lut_lite::decode_mm(depth_image8[r][c]));
                } else {
                    return false;
                }
                const size_t index = static_cast<size_t>(r) * iTFS::lite_max_col + c;
                write_u16_le(&depth_mm[index * sizeof(uint16_t)], saturate_u16(value_mm));
            }
        }
        return true;
    }

    // Amplitude and intensity share the SDK's RAW/LIN/LOG encoding family.
    bool restore_amplitude_raw(const DeviceContext &context,
                               const LiteFrameSnapshot &frame,
                               std::vector<uint8_t> &amplitude_raw) const {
        return restore_amp_int_raw(frame,
                                   iTFS::lite_img_amplitude,
                                   context.output_layout.amplitude_mode,
                                   context.output_layout.amplitude_8bit,
                                   amplitude_raw);
    }

    // Intensity uses the same restoration path and LUT as amplitude.
    bool restore_intensity_raw(const DeviceContext &context,
                               const LiteFrameSnapshot &frame,
                               std::vector<uint8_t> &intensity_raw) const {
        return restore_amp_int_raw(frame,
                                   iTFS::lite_img_intensity,
                                   context.output_layout.intensity_mode,
                                   context.output_layout.intensity_8bit,
                                   intensity_raw);
    }

    // Restore an amplitude or intensity slot to its 16-bit raw scale. LIN_8
    // carries the raw high byte; LOG_8 is inverted by the SDK's shared LUT.
    bool restore_amp_int_raw(const LiteFrameSnapshot &frame,
                             uint8_t img_class,
                             uint16_t mode,
                             bool image_8bit,
                             std::vector<uint8_t> &raw_data) const {
        if (mode == iTFS::packet::info_v3_data_output_stream_off) {
            return false;
        }

        const uint8_t (*image_data8)[iTFS::lite_max_col] = image_8bit ?
            iTFS::get_img_u8_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data), img_class) : nullptr;
        const uint16_t (*image_data16)[iTFS::lite_max_col] = !image_8bit ?
            iTFS::get_img_u16_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data), img_class) : nullptr;
        if (image_data8 == nullptr && image_data16 == nullptr) {
            return false;
        }

        if (mode == iTFS::packet::info_v3_data_output_stream_raw_16bit &&
            image_data16 != nullptr &&
            copy_native_u16_image(&image_data16[0][0], raw_data)) {
            return true;
        }

        for (int r = 0; r < iTFS::lite_max_row; ++r) {
            for (int c = 0; c < iTFS::lite_max_col; ++c) {
                uint16_t value = 0;
                if (mode == iTFS::packet::info_v3_data_output_stream_raw_16bit) {
                    value = image_data16[r][c];
                } else if (mode == iTFS::packet::info_v3_data_output_stream_lin_8bit) {
                    value = static_cast<uint16_t>(image_data8[r][c]) << 8;
                } else if (mode == iTFS::packet::info_v3_data_output_stream_log_8bit) {
                    value = iTFS::amp_int_log8_lut_lite::decode_raw(image_data8[r][c]);
                } else {
                    return false;
                }
                const size_t index = static_cast<size_t>(r) * iTFS::lite_max_col + c;
                write_u16_le(&raw_data[index * sizeof(uint16_t)], value);
            }
        }
        return true;
    }

    // Convert both SDK confidence encodings to the public binary mono8 mask.
    // RAW_16 uses the agreed validity threshold; MASK_1 maps nonzero to 255.
    bool restore_confidence_mask(const DeviceContext &context,
                                 const LiteFrameSnapshot &frame,
                                 std::vector<uint8_t> &confidence_mask) const {
        if (context.output_layout.confidence_mode == iTFS::packet::info_v3_data_output_stream_off) {
            return false;
        }

        const uint8_t (*confidence_image8)[iTFS::lite_max_col] = context.output_layout.confidence_mask1 ?
            iTFS::get_img_u8_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data),
                                 iTFS::lite_img_confidence) : nullptr;
        const uint16_t (*confidence_image16)[iTFS::lite_max_col] =
            context.output_layout.confidence_mode == iTFS::packet::info_v3_data_output_stream_raw_16bit ?
            iTFS::get_img_u16_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data),
                                  iTFS::lite_img_confidence) : nullptr;
        if (confidence_image8 == nullptr && confidence_image16 == nullptr) {
            return false;
        }

        for (int r = 0; r < iTFS::lite_max_row; ++r) {
            for (int c = 0; c < iTFS::lite_max_col; ++c) {
                const bool valid = context.output_layout.confidence_mask1 ? confidence_image8[r][c] != 0 :
                    confidence_image16[r][c] >= confidence_raw_valid_threshold;
                confidence_mask[r * iTFS::lite_max_col + c] = valid ? 255 : 0;
            }
        }
        return true;
    }

    // Translate fixed-point SDK intrinsics into ROS CameraInfo. The SDK image
    // axes are mirrored. Calibration fields are initialized once; steady-state
    // publication updates only the frame timestamp.
    void publish_camera_info(DeviceContext &context, const LiteFrameSnapshot &frame) {
        sensor_msgs::msg::CameraInfo &msg = context.camera_info_msg;
        if (!context.camera_info_initialized) {
            const auto &intrinsic = context.intrinsic;
            const double cx = static_cast<double>(iTFS::lite_max_col - 1) -
                              static_cast<double>(intrinsic.lens_cx) * 1e-5;
            const double cy = static_cast<double>(iTFS::lite_max_row - 1) -
                              static_cast<double>(intrinsic.lens_cy) * 1e-5;
            const double fx = static_cast<double>(intrinsic.lens_fx) * 1e-5;
            const double fy = static_cast<double>(intrinsic.lens_fy) * 1e-5;
            const double k1 = static_cast<double>(intrinsic.lens_kc[0]) * 1e-8;
            const double k2 = static_cast<double>(intrinsic.lens_kc[1]) * 1e-8;
            const double p1 = static_cast<double>(intrinsic.lens_kc[2]) * 1e-8;
            const double p2 = static_cast<double>(intrinsic.lens_kc[3]) * 1e-8;
            const double k3 = static_cast<double>(intrinsic.lens_kc[4]) * 1e-8;
            const double k4 = static_cast<double>(intrinsic.lens_kc[5]) * 1e-8;
            if (std::abs(k4) > std::numeric_limits<double>::epsilon()) {
                RCLCPP_WARN(get_logger(), "SN#%u CameraInfo omits unsupported SDK k4 (r^8).",
                            context.sensor_sn);
            }
            msg.header.frame_id = context.settings.optical_frame_id;
            msg.height = iTFS::lite_max_row;
            msg.width = iTFS::lite_max_col;
            msg.distortion_model = "plumb_bob";
            msg.d = {k1, k2, p1, p2, k3};
            msg.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
            msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
            msg.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
            context.camera_info_initialized = true;
        }
        msg.header.stamp = frame.stamp;
        context.depth_camera_info_pub->publish(msg);
    }

    // Preserve the SDK example's organized 320x240 layout. Depth modes use the
    // reconstruction map; XYZ modes use the three native sensor slots.
    void publish_pointcloud(DeviceContext &context,
                            const LiteFrameSnapshot &frame,
                            const sensor_msgs::msg::Image *amplitude_image) {
        const bool include_amplitude = context.settings.publish_pointcloud_amplitude &&
            context.output_layout.amplitude_mode !=
                iTFS::packet::info_v3_data_output_stream_off;

        sensor_msgs::msg::PointCloud2 &cloud = context.points_msg;
        cloud.header.stamp = frame.stamp;
        cloud.header.frame_id = context.settings.link_frame_id;
        cloud.height = iTFS::lite_max_row;
        cloud.width = iTFS::lite_max_col;
        cloud.is_dense = false;

        // Add amplitude only when that stream was actually received and
        // restored for this same frame.
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        // data_output is immutable for this process, so the PointCloud2 field
        // schema is created once on the first subscribed frame.
        if (!context.points_schema_initialized) {
            if (include_amplitude) {
                modifier.setPointCloud2Fields(4,
                                              "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                              "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                              "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                              "amplitude", 1, sensor_msgs::msg::PointField::FLOAT32);
            } else {
                modifier.setPointCloud2FieldsByString(1, "xyz");
            }
            context.points_schema_initialized = true;
        }
        modifier.resize(iTFS::lite_max_row * iTFS::lite_max_col);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        std::optional<sensor_msgs::PointCloud2Iterator<float>> iter_amplitude;
        if (include_amplitude) {
            iter_amplitude.emplace(cloud, "amplitude");
        }

        const uint8_t (*depth_image8)[iTFS::lite_max_col] = context.output_layout.depth_8bit ?
            iTFS::get_img_u8_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data), iTFS::lite_img_depth) : nullptr;
        const uint16_t (*depth_image16)[iTFS::lite_max_col] = !context.output_layout.depth_8bit ?
            iTFS::get_img_u16_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data), iTFS::lite_img_depth) : nullptr;
        const int8_t (*point_x_image8)[iTFS::lite_max_col] = context.output_layout.depth_8bit ?
            iTFS::get_img_s8_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data), iTFS::lite_img_point_x) : nullptr;
        const int8_t (*point_y_image8)[iTFS::lite_max_col] = context.output_layout.depth_8bit ?
            iTFS::get_img_s8_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data), iTFS::lite_img_point_y) : nullptr;
        const int16_t (*point_x_image16)[iTFS::lite_max_col] = !context.output_layout.depth_8bit ?
            iTFS::get_img_s16_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data), iTFS::lite_img_point_x) : nullptr;
        const int16_t (*point_y_image16)[iTFS::lite_max_col] = !context.output_layout.depth_8bit ?
            iTFS::get_img_s16_ptr(const_cast<iTFS::lite_img_cpy_t *>(&frame.lite_img_data), iTFS::lite_img_point_y) : nullptr;

        // Invalid pixels remain in the organized cloud as NaN rather than
        // changing width/height or silently compacting the point set.
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (int r = 0; r < iTFS::lite_max_row; ++r) {
            for (int c = 0; c < iTFS::lite_max_col; ++c, ++iter_x, ++iter_y, ++iter_z) {
                float x = nan;
                float y = nan;
                float z = nan;

                if (frame.lite_img_data.depth_on &&
                    (depth_image8 != nullptr || depth_image16 != nullptr)) {
                    float depth_m = 0.0f;
                    if (context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_depth_mm_16bit) {
                        depth_m = static_cast<float>(depth_image16[r][c]) * 0.001f;
                    } else if (context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_depth_raw_q16) {
                        depth_m = static_cast<float>(depth_image16[r][c]) * depth_raw_q16_to_m;
                    } else if (context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_depth_lin_8bit) {
                        depth_m = static_cast<float>(depth_image8[r][c]) * depth_lin8_to_m;
                    } else if (context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_depth_log_8bit) {
                        depth_m = static_cast<float>(iTFS::depth_log8_lut_lite::decode_mm(depth_image8[r][c])) * 0.001f;
                    }
                    if (depth_m > 0.0f) {
                        // Same axis conversion as lite_pcl_example.cpp.
                        x = depth_m * context.reconstruction_map.dir[2][r][c];
                        y = -depth_m * context.reconstruction_map.dir[0][r][c];
                        z = -depth_m * context.reconstruction_map.dir[1][r][c];
                    }
                } else if (frame.lite_img_data.xyz_on &&
                           (depth_image8 != nullptr || depth_image16 != nullptr)) {
                    float sensor_x = 0.0f;
                    float sensor_y = 0.0f;
                    float sensor_z = 0.0f;
                    if (context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_xyz_mm_16bit &&
                        point_x_image16 != nullptr && point_y_image16 != nullptr) {
                        sensor_x = static_cast<float>(point_x_image16[r][c]) * 0.001f;
                        sensor_y = static_cast<float>(point_y_image16[r][c]) * 0.001f;
                        sensor_z = static_cast<float>(depth_image16[r][c]) * 0.001f;
                    } else if (context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_xyz_raw_q15_q16 &&
                               point_x_image16 != nullptr && point_y_image16 != nullptr) {
                        sensor_x = static_cast<float>(point_x_image16[r][c]) * xyz_raw_q15_to_m;
                        sensor_y = static_cast<float>(point_y_image16[r][c]) * xyz_raw_q15_to_m;
                        sensor_z = static_cast<float>(depth_image16[r][c]) * depth_raw_q16_to_m;
                    } else if (context.output_layout.depth_mode == iTFS::packet::info_v3_data_output_xyz_lin_8bit &&
                               point_x_image8 != nullptr && point_y_image8 != nullptr) {
                        sensor_x = static_cast<float>(point_x_image8[r][c]) * xyz_lin8_to_m;
                        sensor_y = static_cast<float>(point_y_image8[r][c]) * xyz_lin8_to_m;
                        sensor_z = static_cast<float>(depth_image8[r][c]) * depth_lin8_to_m;
                    }
                    if (sensor_z > 0.0f) {
                        // Same axis conversion as transform_xyz_float().
                        x = sensor_z;
                        y = -sensor_x;
                        z = -sensor_y;
                    }
                }

                *iter_x = x;
                *iter_y = y;
                *iter_z = z;
                if (iter_amplitude) {
                    const size_t idx = static_cast<size_t>(r) * iTFS::lite_max_col + c;
                    const bool amplitude_valid = amplitude_image != nullptr &&
                        amplitude_image->data.size() >= (idx + 1) * sizeof(uint16_t);
                    **iter_amplitude = amplitude_valid ?
                        static_cast<float>(read_u16_le(
                            &amplitude_image->data[idx * sizeof(uint16_t)])) : nan;
                    ++(*iter_amplitude);
                }
            }
        }
        context.points_pub->publish(cloud);
    }

    // Publish base->link and link->optical together on /tf_static.
    void publish_static_tf(const DeviceContext &context) {
        const rclcpp::Time stamp = now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(make_transform(stamp,
                                            context.settings.parent_frame_id,
                                            context.settings.link_frame_id,
                                            context.settings.mount_translation_x,
                                            context.settings.mount_translation_y,
                                            context.settings.mount_translation_z,
                                            context.settings.mount_rotation_roll,
                                            context.settings.mount_rotation_pitch,
                                            context.settings.mount_rotation_yaw));
        transforms.push_back(make_transform(stamp,
                                            context.settings.link_frame_id,
                                            context.settings.optical_frame_id,
                                            context.settings.optical_translation_x,
                                            context.settings.optical_translation_y,
                                            context.settings.optical_translation_z,
                                            context.settings.optical_rotation_roll,
                                            context.settings.optical_rotation_pitch,
                                            context.settings.optical_rotation_yaw));
        static_tf_broadcaster_->sendTransform(transforms);
    }

    std::unique_ptr<iTFS::LITE> lite_;
    std::array<std::unique_ptr<DeviceContext>, iTFS::max_device> contexts_;
    std::array<std::atomic<DeviceContext *>, iTFS::max_device> context_by_idx_;
    std::array<std::atomic<bool>, iTFS::max_device> context_init_failed_;
    std::mutex context_init_mutex_;
    DeviceSettings default_settings_;
    std::string broadcast_ip_;
    std::string listening_ip_;
    int listening_port_ = iTFS::user_data_port;
    std::array<uint8_t, 4> broadcast_ip_bytes_{};
    std::array<uint8_t, 4> listening_ip_bytes_{};
    int diagnostics_qos_depth_ = 10;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::TimerBase::SharedPtr subscription_timer_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<LiteCoreNode>());
    } catch (const std::exception &exc) {
        std::fprintf(stderr, "[ERROR] ilidar_lite_core: %s\n", exc.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
