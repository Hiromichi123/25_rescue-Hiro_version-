#ifndef CAR_NODE_H
#define CAR_NODE_H

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "ros_tools/msg/lidar_pose.hpp"
#include "core/msg/calibrate.hpp"
#include "core/msg/ball_position.hpp"
#include <std_msgs/msg/int32.hpp>

class ball;
class target;

class car : public rclcpp::Node {
public:
    float x, y, yaw; // 车全局位置

    car();
    void car_init();

private:
    std::shared_ptr<std::thread> ball_thread;
    std::shared_ptr<std::thread> spin_thread;
    std::shared_ptr<ball> ball_node;
    std::shared_ptr<target> front_target;
    
    std::shared_ptr<rclcpp::Rate> rate;
    rclcpp::Subscription<ros_tools::msg::LidarPose>::SharedPtr lidar_sub;
    rclcpp::Subscription<core::msg::BallPosition>::SharedPtr ball_sub;
    rclcpp::Subscription<core::msg::Calibrate>::SharedPtr calibrate_sub;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pos_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr calibrate_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor_pub;
    
    std::shared_ptr<ros_tools::msg::LidarPose> lidar_pose_data;
    std::shared_ptr<core::msg::BallPosition> ball_pos;
    std::shared_ptr<core::msg::Calibrate> calibrate_msg;
    std::shared_ptr<geometry_msgs::msg::Twist> target_pose;
    std::shared_ptr<geometry_msgs::msg::Vector3> calibrate_vel;
    std_msgs::msg::Int32 motor_msg;

    friend class ball;
    friend class target;

    void lidar_pose_cb(const ros_tools::msg::LidarPose::SharedPtr msg);
    void ball_pos_cb(const core::msg::BallPosition::SharedPtr msg);
    void calibrate_cb(core::msg::Calibrate::SharedPtr msg);
};
#endif
