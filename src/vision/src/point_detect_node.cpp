// ROS2头
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "vision/msg/lidar_ball_position.hpp"
#include "ros_tools/msg/lidar_pose.hpp"
// pcl头
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>

class SphereDetectionNode : public rclcpp::Node {
public:
    SphereDetectionNode() : Node("vision_node") {
        pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/pointcloud2", 10, std::bind(&SphereDetectionNode::pointcloud_callback, this, std::placeholders::_1));

        /* // 订阅位置信息
        lidar_pose_sub = this->create_subscription<ros_tool::LidarPose>(
            "/lidar_data", 10, std::bind(&SphereDetectionNode::lidar_pose_callback, this, std::placeholders::_1));*/
        
        // 发布检测到的球心坐标和全局位置
        sphere_pub = this->create_publisher<vision::msg::LidarBallPosition>("/detected_sphere_centers", 10);
        // global_sphere_pub = this->create_publisher<vision::msg::LidarBallPosition>("/lidar_ball_position", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
    // rclcpp::Subscription<ros_tool::LidarPose>::SharedPtr lidar_pose_sub;
    rclcpp::Publisher<vision::msg::LidarBallPosition>::SharedPtr sphere_pub;
    // rclcpp::Publisher<vision::msg::LidarBallPosition>::SharedPtr global_sphere_pub;

    vision::msg::LidarBallPosition sphere_position_msg;
    // vision::msg::LidarBallPosition global_position_msg;

    bool ball_found = false;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // 1.直通滤波
        pcl::PassThrough<pcl::PointXYZ> pass;
        std::vector<std::tuple<std::string, float, float>> filters = {
            {"x", -1.5, 1.5},
            {"y", -1.5, 1.5},
            {"z", -0.2, 0.0}
        };
        for (const auto& filter : filters) {// 这非常python
            const auto& [axis, min, max] = filter;
            pass.setFilterFieldName(axis);
            pass.setFilterLimits(min, max);
            pass.filter(*cloud);
        }

        // 2.体素滤波
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.005f, 0.005f, 0.005f);
        voxel_grid.filter(*cloud);

        // 3.聚类
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices; // 结果索引
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.04);
        ec.setMinClusterSize(2);
        ec.setMaxClusterSize(100);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 4.检测球体
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : indices.indices)
                cloud_cluster->push_back((*cloud)[idx]);

            // 使用 SACSegmentation 进行球体检测
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

            // 设置模型参数
            seg.setModelType(pcl::SACMODEL_SPHERE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.02); // 内点阈值
            seg.setRadiusLimits(0.015, 0.025);  // 球半径范围
            seg.setMaxIterations(200); // 最大迭代次数
            seg.setInputCloud(cloud_cluster);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() > 10) {
                ball_found = true;

                // 计算球心局部坐标
                float sphere_x = coefficients->values[0];
                float sphere_y = coefficients->values[1];
                // 计算全局坐标
                // float global_x = car_x_ + (sphere_x * cos(car_yaw_) - sphere_y * sin(car_yaw_));
                // float global_y = car_y_ + (sphere_x * sin(car_yaw_) + sphere_y * cos(car_yaw_));

                // 填充局部和全局坐标消息
                sphere_position_msg.x = sphere_x;
                sphere_position_msg.y = sphere_y;
                sphere_position_msg.found = true;
                // global_position_msg.x = global_x;
                // global_position_msg.y = global_y;
                // global_position_msg.found = true;

                // 发布局部和全局坐标
                sphere_pub->publish(sphere_position_msg);
                // global_sphere_pub->publish(global_position_msg);
            }
        }

        if (!ball_found) {
            sphere_position_msg.found = false;
            // global_position_msg.found = false;

            sphere_pub->publish(sphere_position_msg);
            // global_sphere_pub->publish(global_position_msg);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SphereDetectionNode>());
    rclcpp::shutdown();
    return 0;
}