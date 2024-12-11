#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ros_tools/msg/lidar_pose.hpp"

class LidarDataNode : public rclcpp::Node
{
public:
    rclcpp::Publisher<ros_tools::msg::LidarPose>::SharedPtr lidar_pub;  // 发布 LidarPose 类型消息
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;  // 订阅 Odometry 消息

    LidarDataNode() : Node("lidar_data_node")
    {
        // 创建 LidarPose 发布者
        lidar_pub = this->create_publisher<ros_tools::msg::LidarPose>("lidar_data", 10);
        RCLCPP_INFO(this->get_logger(), "Publisher for 'lidar_data' created");

        // 创建 Odometry 订阅者
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&LidarDataNode::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Odometry subscription successful.");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Odometry data");

        const auto &pose = msg->pose.pose;

        // 转换四元数为欧拉角
        double x = pose.position.x;
        double y = pose.position.y;
        double z = pose.position.z;
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 确保欧拉角为正值（根据需要调整）
        if (roll < 0) roll += 2 * M_PI;
        if (pitch < 0) pitch += 2 * M_PI;
        if (yaw < 0) yaw += 2 * M_PI;

        // 创建并发布 LidarPose 消息
        ros_tools::msg::LidarPose output;
        output.x = x;
        output.y = y;
        output.z = z;
        output.roll = roll;
        output.pitch = pitch;
        output.yaw = yaw;

        // 发布 LidarPose 数据
        lidar_pub->publish(output);

        RCLCPP_INFO(this->get_logger(), "Position=(%.2f, %.2f, %.2f), Orientation=(%.2f, %.2f, %.2f) rad",
                    x, y, z, roll, pitch, yaw);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarDataNode>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Spinning node...");
    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down...");
    rclcpp::shutdown();
    return 0;
}