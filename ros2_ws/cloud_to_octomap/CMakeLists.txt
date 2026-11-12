cmake_minimum_required(VERSION 3.8)
project(cloud_to_octomap)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)

add_executable(cloud_to_octomap_node src/cloud_to_octomap_node.cpp)
ament_target_dependencies(cloud_to_octomap_node
  rclcpp sensor_msgs octomap octomap_msgs octomap_ros)

install(TARGETS cloud_to_octomap_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()