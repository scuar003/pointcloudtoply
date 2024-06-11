#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

//Alias
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PCLcloud = pcl::PointCloud<pcl::PointXYZ>;


class PCLConverter : public rclcpp::Node
{
public:
    PCLConverter() : Node("pcl_converter_node")
    {
        // Create a subscription to the PointCloud2 topic
        subscription_ = this->create_subscription<PointCloud2>("points", 10,std::bind(&PCLConverter::toPCL, this, std::placeholders::_1));
    }

private:
    void toPCL(const PointCloud2::SharedPtr msg)
    {
        // Convert the ROS PointCloud2 message to a PCL point cloud
        PCLcloud pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        //process the point cloud here
        toPLY(pcl_cloud, "output.ply");
    }
    void toPLY(const PCLcloud &cloud, const std::string &filename) {
        if (pcl::io::savePLYFile(filename, cloud) == -1){
            std::cout << "failed to save pointcloud" << std::endl;
        } else {
            std::cout << "Succesfully saved pointcloud" << std::endl;
        }
    }

    rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
