cmake_minimum_required(VERSION 3.5)
project(pointcloudtoply)

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

# Add the executable
add_executable(pcl_converter_node src/pctoply.cpp)

# Link libraries
ament_target_dependencies(pcl_converter_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pcl_converter_node ${PCL_LIBRARIES})

# Install targets
install(TARGETS
  pcl_converter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
