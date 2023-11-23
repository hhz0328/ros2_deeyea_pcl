#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

class PclSub : public rclcpp::Node
{
public:
    PclSub() : Node("pcl_subscriber")
    {
        RCLCPP_INFO(this->get_logger(), "begin!");
        // 创建订阅者
        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth_registered/points", 10, 
            std::bind(&PclSub::topic_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        
        RCLCPP_INFO(this->get_logger(), "Received a message.");

        // 从ROS消息转换为PCL点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *cloud);

        // 保存点云到文件
        pcl::io::savePCDFileASCII ("/home/foxy/ros2_deeyea_final/output.pcd", *cloud);
        RCLCPP_INFO(this->get_logger(), "PointCloud saved: points_size(%d, %d)", msg->height, msg->width);

           
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //spin函数，传入节点对象指针
    rclcpp::spin(std::make_shared<PclSub>());
    rclcpp::shutdown();
    return 0;
}

