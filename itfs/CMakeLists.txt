cmake_minimum_required(VERSION 3.8)
project(ilidar_itfs_ros2 VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

add_executable(ilidar_core
  src/ilidar_core.cpp
  src/ilidar.cpp
)

target_include_directories(ilidar_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(ilidar_core PRIVATE -Wno-register)
endif()

ament_target_dependencies(ilidar_core
  ament_index_cpp
  diagnostic_msgs
  geometry_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

target_link_libraries(ilidar_core Threads::Threads)

install(TARGETS ilidar_core
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/ilidar_viewer
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/iTFS-110.dat
  src/iTFS-80.dat
  DESTINATION share/${PROJECT_NAME}/dat
)

install(FILES README.md CHANGELOG.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
