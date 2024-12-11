#ifndef BALL_NODE_H
#define BALL_NODE_H

#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ros_tools/msg/lidar_pose.hpp"

#define pi 3.14

class car;

//--------------------------------------------------------------------------------------------------
// 目标点类
class target : public rclcpp::Node {
public:
    float x, y, yaw;
    bool reached;

    target(float ball_x, float ball_y, car* car_node);
    target(car* car_node);
    target(float x, float y, float yaw, car* car_node);

    void drive_to_target();
    bool pos_check(std::shared_ptr<ros_tools::msg::LidarPose> lidar_pose_data, double distance = 0.1);
    bool pos_check(std::shared_ptr<ros_tools::msg::LidarPose> lidar_pose_data, double distance_x, double distance_y);

private:
    car* car_node;
    std::shared_ptr<rclcpp::Rate> rate;
};

class ball : public rclcpp::Node {
public:
    float x, y;
    std::shared_ptr<target> calibrate_target;
    car* car_node;

    ball(float x, float y, car* car_node);
    ball(car* car_node);
    
    void ball_init();
    void pick_up();

private:
    std::shared_ptr<rclcpp::Rate> rate;
};

#endif
