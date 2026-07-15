cmake_minimum_required(VERSION 3.8)
project(ilidar_lite_ros2 VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

# Keep the copied SDK translation unit and ROS wrapper in one executable. SDK
# headers remain private because this package exposes ROS interfaces, not a C++
# library API.
add_executable(ilidar_lite_core
  src/ilidar_lite_core.cpp
  src/ilidar_lite.cpp
)

target_include_directories(ilidar_lite_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(ilidar_lite_core PRIVATE -Wno-register)
endif()

ament_target_dependencies(ilidar_lite_core
  diagnostic_msgs
  geometry_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

target_link_libraries(ilidar_lite_core
  Threads::Threads
)

# Launch, parameter, and RViz files are resolved from the installed package
# share directory by both launch descriptions.
install(TARGETS ilidar_lite_core
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/ilidar_lite_viewer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md CHANGELOG.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
