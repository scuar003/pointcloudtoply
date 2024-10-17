#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <string>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PCLcloud = pcl::PointCloud<pcl::PointXYZRGB>;  // Use PointXYZRGB to include color information

class PCLConverter : public rclcpp::Node
{
public:
    PCLConverter() : Node("pcl_converter_node")
    {
        // Create a subscription to the PointCloud2 topic
        subscription_ = create_subscription<PointCloud2>("/camera/camera/depth/color/points", 10, std::bind(&PCLConverter::toPCL, this, std::placeholders::_1));
        
        // Setup UDP socket
        UDP_IP = "0.0.0.0";
        UDP_PORT = 5007;
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error creating socket");
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(UDP_PORT);
        server_addr.sin_addr.s_addr = inet_addr(UDP_IP.c_str());

        // Bind the socket
        if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;  // 0.1 seconds
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    }

    ~PCLConverter()
    {
        if (sock >= 0)
        {
            close(sock);
        }
    }

private:
    void toPCL(const PointCloud2::SharedPtr msg)
    {
        // Convert the ROS PointCloud2 message to a PCL point cloud with RGB
        PCLcloud pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Process the point cloud and send via UDP
        sendPointCloudUDP(pcl_cloud);
    }

    void sendPointCloudUDP(const PCLcloud &cloud)
    {
        std::vector<float> data_buffer;

        // Iterate through all points in the cloud and extract XYZ and RGB
        for (const auto &point : cloud.points)
        {
            // Add XYZ coordinates
            data_buffer.push_back(point.x);
            data_buffer.push_back(point.y);
            data_buffer.push_back(point.z);

            // Convert RGB from float (used by PCL) to individual R, G, B values
            uint32_t rgb = *reinterpret_cast<const uint32_t*>(&point.rgb);
            float r = (rgb >> 16) & 0x0000FF;
            float g = (rgb >> 8) & 0x0000FF;
            float b = (rgb) & 0x0000FF;

            // Add RGB values as floats
            data_buffer.push_back(r / 255.0f);  // Normalize to 0-1
            data_buffer.push_back(g / 255.0f);  // Normalize to 0-1
            data_buffer.push_back(b / 255.0f);  // Normalize to 0-1
        }

        // Send the data buffer as floats via UDP
        
        std::cout<<"Data:"<< data_buffer.data()<<std::endl;
        ssize_t bytes_sent = sendto(sock, data_buffer.data(), data_buffer.size() * sizeof(float), 0,
                                    (struct sockaddr *)&server_addr, sizeof(server_addr));

        if (bytes_sent < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data via UDP");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Successfully sent point cloud via UDP, size: %ld bytes", bytes_sent);
        }
    }

    std::string UDP_IP;
    int UDP_PORT;
    int sock;
    struct sockaddr_in server_addr;
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

