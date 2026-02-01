cmake_minimum_required(VERSION 3.8)
project(camera_latency_benchmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

add_library(camera_latency_benchmark_nodes SHARED
  src/fake_camera_node.cpp
  src/latency_probe_node.cpp
)

target_include_directories(camera_latency_benchmark_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(camera_latency_benchmark_nodes
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
)

rclcpp_components_register_nodes(camera_latency_benchmark_nodes
  "camera_latency_benchmark::FakeCameraNode"
  "camera_latency_benchmark::LatencyProbeNode"
)

add_executable(fake_camera_node src/fake_camera_main.cpp)
ament_target_dependencies(fake_camera_node rclcpp)
target_link_libraries(fake_camera_node camera_latency_benchmark_nodes)

add_executable(latency_probe_node src/latency_probe_main.cpp)
ament_target_dependencies(latency_probe_node rclcpp)
target_link_libraries(latency_probe_node camera_latency_benchmark_nodes)

install(TARGETS
  camera_latency_benchmark_nodes
  fake_camera_node
  latency_probe_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
