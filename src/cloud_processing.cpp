#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>

// Alias
using PointT = pcl::PointXYZ;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PCLcloud = pcl::PointCloud<PointT>;
using VoxelGrid = pcl::VoxelGrid<PointT>;

class PCLConverter : public rclcpp::Node
{
public:
    PCLConverter() : Node("pcl_converter_node")
    {
        // Create a subscription to the PointCloud2 topic
        sub_pointcloud = create_subscription<PointCloud2>("points", 10, std::bind(&PCLConverter::toPCL, this, std::placeholders::_1));
        pub_process_cloud = create_publisher<PointCloud2>("processed_points", 10);
        frame_counter = 0;
    }

private:
    void toPCL(const PointCloud2::SharedPtr msg)
    {
        // Convert the ROS PointCloud2 message to a PCL point cloud
        PCLcloud::Ptr og_pcl_cloud(new PCLcloud);
        PCLcloud::Ptr process_pcl_cloud(new PCLcloud);
        pcl::fromROSMsg(*msg, *og_pcl_cloud);

        // Process the point cloud here
        downsampleFilter(og_pcl_cloud, process_pcl_cloud);
        PointCloud2 ros2_cloud;
        toROS2(process_pcl_cloud, ros2_cloud);
        pub_process_cloud -> publish(ros2_cloud);

    }
    void downsampleFilter(const PCLcloud::Ptr &og_cloud, const PCLcloud::Ptr &process_cloud) {
        auto leaf_size = 0.05f;
        VoxelGrid sor;
        sor.setInputCloud(og_cloud);
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);
        sor.filter(*process_cloud);

    }

    void toROS2(const PCLcloud::Ptr &process_pcl_cloud, PointCloud2 &ros2_cloud) {
        pcl::toROSMsg(*process_pcl_cloud, ros2_cloud);
        ros2_cloud.header.frame_id = "laser_data_frame"; // Set the frame ID
        ros2_cloud.header.stamp = this->get_clock()-> now();
    }

    int frame_counter;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_pointcloud;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_process_cloud;
};

int main(int argc, char *argv[])
{
   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
