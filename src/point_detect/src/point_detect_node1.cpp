#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include "point_detect/msg/lidar_ball_position.hpp"

class SphereDetectionNode : public rclcpp::Node {
public:
    SphereDetectionNode() : Node("point_detect_node") {
        // 订阅来自点云话题的数据
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/pointcloud2", 10, std::bind(&SphereDetectionNode::pointcloud_callback, this, std::placeholders::_1));
        
        // 发布检测到的球心坐标
        sphere_center_pub_ = this->create_publisher<point_detect::msg::LidarBallPosition>("/detected_sphere_centers", 10);
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // 对点云进行直通滤波，限制x轴范围
        pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-1.5, 1.5);
        pass.filter(*cloud_filtered_x);

        // 对点云进行直通滤波，限制y轴范围
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>());
        pass.setInputCloud(cloud_filtered_x);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-1.5, 1.5);
        pass.filter(*cloud_filtered_y);

        // 对点云进行直通滤波，限制z轴范围在地面（-0.1m）以上
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>());
        pass.setInputCloud(cloud_filtered_y);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.12, 0);  // 地面为-0.1m以上
        pass.filter(*cloud_filtered_z);

        // 对点云进行聚类提取
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered_z);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.04);  // 设置聚类容差
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(10000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered_z);
        ec.extract(cluster_indices);

        bool ball_found = false;
        point_detect::msg::LidarBallPosition sphere_position_msg;

        // 遍历所有聚类，找到符合条件的聚类
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : indices.indices) {
                cloud_cluster->push_back((*cloud_filtered_z)[idx]);
            }

            // 获取聚类中的最高点
            float max_z = -std::numeric_limits<float>::infinity();
            pcl::PointXYZ highest_point;
            for (const auto& point : cloud_cluster->points) {
                if (point.z > max_z) {
                    max_z = point.z;
                    highest_point = point;
                }
            }

            // 判断最高点是否为距离地面4cm左右
            if (max_z >= -0.09 && max_z <= -0.05) {  // 地面为-0.1m，4cm左右的范围为-0.06到-0.04
                ball_found = true;
                // 填充球的坐标信息（使用最高点作为球的位置）
                sphere_position_msg.x = highest_point.x;
                sphere_position_msg.y = highest_point.y;
                sphere_position_msg.found = true;

                // 发布球心信息
                sphere_center_pub_->publish(sphere_position_msg);
            }
        }

        // 如果未检测到球，发布未检测到的标志
        if (!ball_found) {
            sphere_position_msg.found = false;
            sphere_center_pub_->publish(sphere_position_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<point_detect::msg::LidarBallPosition>::SharedPtr sphere_center_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SphereDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
