#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>//直通滤波
#include <pcl/filters/voxel_grid.h>//下采样，创建三维体素栅格
#include <pcl/filters/uniform_sampling.h>//下采样，减少点的数量以便于处理，但同时希望尽可能保留原始点云的结构和形状
#include <pcl/filters/statistical_outlier_removal.h>//移除离群点
#include <pcl/filters/fast_bilateral.h>//双边滤波，处理深度相机有序点云
#include <pcl/features/integral_image_normal.h>  //法线估计类
#include <pcl/features/normal_3d.h>
#include <iostream>

//只接收一次深度相机的/camera/depth_registered/points话题并读取所有点云信息
//转化成PCLXYZ格式
//特征提取,法向量

class PclSub : public rclcpp::Node
{
public:
    PclSub() : Node("pcl_subscriber"), cloud_viewer_("PointCloud Viewer"),received_(false)
    {
        RCLCPP_INFO(this->get_logger(), "begin!");
        // 创建订阅者
        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth_registered/points", 10, 
            std::bind(&PclSub::topic_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    pcl::visualization::CloudViewer cloud_viewer_;

    bool received_; // 用于跟踪是否已接收消息

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (received_) {
            return; // 如果已经接收过消息，直接返回
        }
        
        RCLCPP_INFO(this->get_logger(), "Received a message.");

        // 从ROS消息转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered4 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        //双边滤波，去除噪声的同时保留边缘信息，适用于对细节敏感的应用场景
        pcl::FastBilateralFilter<pcl::PointXYZ> fbFilter;
        fbFilter.setInputCloud(cloud);
        float sigmaS = 10.0f;         // 空间标准差
        fbFilter.setSigmaS(sigmaS);   // 设置滤波器的空间范围参数
        float sigmaR = 0.05f;         // 强度标准差
        fbFilter.setSigmaR(sigmaR);   // 设置滤波器的强度范围参数
        fbFilter.filter(*cloud_filtered1);

        // 使用体素化网格(VoxelGrid)进行下采样
        pcl::VoxelGrid<pcl::PointXYZ> grid; //创建滤波对象
        const float leaf = 0.002f; 
        grid.setLeafSize(leaf, leaf, leaf); // 设置体素体积
        grid.setInputCloud(cloud_filtered1); // 设置点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxelResult(new pcl::PointCloud<pcl::PointXYZ>);
        grid.filter(*cloud_filtered2); // 执行滤波，输出结果

        // 直通滤波器
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_filtered2); 
        pass.setFilterFieldName ("x"); 
        pass.setFilterLimits (-0.25, 0.25);        //设置在过滤字段的范围
        //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内 是否保存滤波的限制范围内的点云，默认为false，保存限制范围内点云，true时候是相反。
        pass.filter (*cloud_filtered3);            //执行滤波，保存过滤结果

        //离群点去除
        // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
        //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
        sor.setInputCloud (cloud_filtered3);               //设置待滤波的点云
        sor.setMeanK (50);                               //设置在进行统计时考虑查询点临近点数
        sor.setStddevMulThresh (1.0);                      //设置判断是否为离群点的阀值
        sor.filter (*cloud_filtered4);                    

        // 计算法向量
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> nest;
        //nest.setRadiusSearch(0.01); // 设置拟合时邻域搜索半径，最好用模型分辨率的倍数
        nest.setKSearch(50); // 设置拟合时采用的点数
        nest.setInputCloud(cloud_filtered4);
        pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
        nest.compute(*normals);
        for (size_t i = 0; i < cloud_filtered4->points.size(); ++i)
        {	// 生成时只生成了法向量，没有将原始点云信息拷贝，为了显示需要复制原信息
            // 也可用其他方法进行连接，如：pcl::concatenateFields
            normals->points[i].x = cloud_filtered4->points[i].x;
            normals->points[i].y = cloud_filtered4->points[i].y;
            normals->points[i].z = cloud_filtered4->points[i].z;
        }
        // 显示
        pcl::visualization::PCLVisualizer viewer;
        viewer.addPointCloud(cloud_filtered4, "cloud_filtered4");
        int level = 50; // 多少条法向量集合显示成一条
        float scale = 0.01; // 法向量长度
        viewer.addPointCloudNormals<pcl::PointNormal>(normals, level, scale, "normals");

        // 保存点云到文件
        pcl::io::savePCDFileASCII ("/home/foxy/ros2_deeyea_final/output_twice_orignal.pcd", *cloud);
        pcl::io::savePCDFileASCII ("/home/foxy/ros2_deeyea_final/output_twice_1.pcd", *cloud_filtered1);
        pcl::io::savePCDFileASCII ("/home/foxy/ros2_deeyea_final/output_twice_2.pcd", *cloud_filtered2);
        pcl::io::savePCDFileASCII ("/home/foxy/ros2_deeyea_final/output_twice_3.pcd", *cloud_filtered3);
        pcl::io::savePCDFileASCII ("/home/foxy/ros2_deeyea_final/output_twice_4.pcd", *cloud_filtered4);
        pcl::io::savePCDFileASCII ("/home/foxy/ros2_deeyea_final/output_twice_features.pcd", *normals);
        RCLCPP_INFO(this->get_logger(), "PointCloud saved: points_size(%d, %d)", msg->height, msg->width);

        received_ = true; // 标记已接收消息
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PclSub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
