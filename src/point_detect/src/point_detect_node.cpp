// ROS2头
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "point_detect/msg/lidar_ball_position.hpp"
// #include "msg_tools/msg/lidar_pose.hpp"
// pcl头
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

class SphereDetectionNode : public rclcpp::Node {
public:
    SphereDetectionNode() : Node("vision_node") {
        pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/pointcloud2", 10, std::bind(&SphereDetectionNode::pointcloud_callback, this, std::placeholders::_1));

        /* // 订阅位置信息
        lidar_pose_sub = this->create_subscription<ros_tool::LidarPose>(
            "/lidar_data", 10, std::bind(&SphereDetectionNode::lidar_pose_callback, this, std::placeholders::_1));*/
        
        // 发布检测到的球心坐标和全局位置
        sphere_pub = this->create_publisher<point_detect::msg::LidarBallPosition>("/detected_sphere_centers", 10);
        // global_sphere_pub = this->create_publisher<point_detect::msg::LidarBallPosition>("/lidar_ball_position", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
    // rclcpp::Subscription<ros_tool::LidarPose>::SharedPtr lidar_pose_sub;
    rclcpp::Publisher<point_detect::msg::LidarBallPosition>::SharedPtr sphere_pub;
    // rclcpp::Publisher<point_detect::msg::LidarBallPosition>::SharedPtr global_sphere_pub;

    point_detect::msg::LidarBallPosition sphere_position_msg;
    // point_detect::msg::LidarBallPosition global_position_msg;

    bool ball_found = false;

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        // 3. 聚类 (基于欧几里得距离)
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.02); // 聚类容忍度
        ec.setMinClusterSize(4);     // 聚类最小点数
        ec.setMaxClusterSize(1000);   // 聚类最大点数
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 4. 筛选点数小于100的聚类
        std::vector<pcl::PointIndices> valid_clusters;
        for (const auto& indices : cluster_indices) {
            if (indices.indices.size() <= 100) {
                valid_clusters.push_back(indices);
            }
        }

        // 如果没有有效聚类，跳过后续处理
        if (valid_clusters.empty()) {
            RCLCPP_WARN(this->get_logger(), "没有找到符合条件的聚类");
            return;
        }

        // 4. 基于反射率 (intensity) 筛选点
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
            for (const auto& idx : indices.indices) {
                cluster->points.push_back(cloud->points[idx]);
            }

            // 在每个聚类内部，根据反射率筛选点
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cluster(new pcl::PointCloud<pcl::PointXYZI>());
            for (const auto& point : cluster->points) {
                if (point.intensity >= 0 && point.intensity <= 100) {
                    filtered_cluster->points.push_back(point);
                }
            }

            // 合并筛选后的聚类点
            *filtered_cloud += *filtered_cluster;
        }

        // 如果筛选后的点云为空，跳过后续处理
        if (filtered_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "没有找到符合条件的点");
            ball_found = false;
            sphere_position_msg.found = false;
            sphere_position_msg.x = 0;
            sphere_position_msg.y = 0;
            sphere_pub->publish(sphere_position_msg);
            return;
        }

        // 5. 计算质心，认为球的质心是球心
        pcl::CentroidPoint<pcl::PointXYZI> centroid;
        for (const auto& point : filtered_cloud->points) {
            centroid.add(point);
        }

        pcl::PointXYZI center;
        centroid.get(center);

        // 6. 填充坐标消息
        sphere_position_msg.x = center.x;
        sphere_position_msg.y = center.y;
        sphere_position_msg.found = true;

        // 7. 发布球心坐标
        sphere_pub->publish(sphere_position_msg);

        // 打印发布的消息
        RCLCPP_INFO(this->get_logger(), "发布球心坐标: x=%.2f, y=%.2f, found=%s", sphere_position_msg.x, sphere_position_msg.y, sphere_position_msg.found ? "true" : "false");

        /* // 3.聚类
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices; // 结果索引
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(1000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 4.处理聚类结果，判断是否为球
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : indices.indices)
                cloud_cluster->push_back((*cloud)[idx]);

            // 如果聚类点数在(10, 100)之间，认为是球体
            if (indices.indices.size() > 10 && indices.indices.size() < 100) {
                ball_found = true;

                // 计算球心坐标，使用聚类的质心作为球心坐标
                pcl::CentroidPoint<pcl::PointXYZ> centroid;
                for (const auto& point : *cloud_cluster)
                    centroid.add(point);

                pcl::PointXYZ center;
                centroid.get(center);

                // 填充坐标消息
                sphere_position_msg.x = center.x;
                sphere_position_msg.y = center.y;
                sphere_position_msg.found = true;

                // 发布球心坐标
                sphere_pub->publish(sphere_position_msg);
            }
            else
            {
                ball_found = false;
                sphere_position_msg.found = false;
                sphere_pub->publish(sphere_position_msg);
            }
        }

        if (!ball_found) {
            sphere_position_msg.found = false;
            sphere_pub->publish(sphere_position_msg);
        } */

        /* // 3.计算PFH特征
        pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
        pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_features(new pcl::PointCloud<pcl::PFHSignature125>());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

        // 计算法线
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setInputCloud(cloud);
        normal_estimator.setKSearch(50);
        normal_estimator.compute(*normals);

        pfh.setInputCloud(cloud);
        pfh.setInputNormals(normals);
        pfh.setRadiusSearch(0.05);
        pfh.compute(*pfh_features);

        // 4.使用RANSAC拟合球体
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_SPHERE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.02); // 内电阈值
        seg.setRadiusLimits(0.015, 0.025);  // 球半径范围
        seg.setMaxIterations(200); // 最大迭代次数
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // 5.发布球心坐标
        if (inliers->indices.size() > 10) {
            ball_found = true;

            // 计算球心局部坐标
            float sphere_x = coefficients->values[0];
            float sphere_y = coefficients->values[1];

            // 填充坐标消息
            sphere_position_msg.x = sphere_x;
            sphere_position_msg.y = sphere_y;
            sphere_position_msg.found = true;

            // 发布球心坐标
            sphere_pub->publish(sphere_position_msg);
        } else {
            ball_found = false;
            sphere_position_msg.found = false;
            sphere_pub->publish(sphere_position_msg);
        }

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
            else
            {
                ball_found = false;
                sphere_position_msg.found = false;
                sphere_pub->publish(sphere_position_msg);
            }
        }

        if (!ball_found) {
            sphere_position_msg.found = false;
            // global_position_msg.found = false;

            sphere_pub->publish(sphere_position_msg);
            // global_sphere_pub->publish(global_position_msg);
        } */
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SphereDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
