#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <filesystem>
#include <string>

// Alias
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PCLcloud = pcl::PointCloud<pcl::PointXYZ>;

class PCLConverter : public rclcpp::Node
{
public:
    PCLConverter(const std::string &save_dir) : Node("pcl_converter_node"), save_directory(save_dir)
    {
        // Create a subscription to the PointCloud2 topic
        subscription_ = create_subscription<PointCloud2>("points", 10, std::bind(&PCLConverter::toPCL, this, std::placeholders::_1));
        frame_counter = 0;

        // Create directory if it doesn't exist
        if (!std::filesystem::exists(save_directory))
        {
            std::filesystem::create_directories(save_directory);
        }
    }

private:
    void toPCL(const PointCloud2::SharedPtr msg)
    {
        // Convert the ROS PointCloud2 message to a PCL point cloud
        PCLcloud pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Process the point cloud here

        std::string filename = save_directory + "/output" + std::to_string(frame_counter) + ".ply";
        toPLY(pcl_cloud, filename);
    }

    void toPLY(const PCLcloud &cloud, const std::string &filename)
    {
        if (pcl::io::savePLYFile(filename, cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s", filename.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Successfully saved point cloud to %s", filename.c_str());
        }
        frame_counter += 1;
    }

    int frame_counter;
    std::string save_directory;
    rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: pcl_converter_node <save_directory>" << std::endl;
        return 1;
    }

    std::string save_directory = argv[1];
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLConverter>(save_directory);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
