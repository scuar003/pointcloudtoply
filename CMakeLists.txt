cmake_minimum_required(VERSION 3.5)
project(pointcloudtoply)

# Set the policy to suppress the warning
cmake_policy(SET CMP0074 NEW)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Add the first executable for pcl_converter_node
add_executable(pcl_converter_node src/pctoply.cpp)

# Link libraries to pcl_converter_node
ament_target_dependencies(pcl_converter_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pcl_converter_node ${PCL_LIBRARIES})

# Add the second executable for pc_udp_sender
add_executable(pc_udp_sender src/pc_udp_sender.cpp)

# Link libraries to pc_udp_sender
ament_target_dependencies(pc_udp_sender rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pc_udp_sender ${PCL_LIBRARIES})

# Install targets
install(TARGETS
  pc_udp_sender
  pcl_converter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
