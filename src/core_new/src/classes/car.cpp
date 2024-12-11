#include<car.h>
#include<ball.h>

class ball;
class target;

car::car() : Node("car_node") {
    rate = std::make_shared<rclcpp::Rate>(20.0);
    lidar_sub = this->create_subscription<ros_tools::msg::LidarPose>("lidar_data", 10, std::bind(&car::lidar_pose_cb, this, std::placeholders::_1));
    ball_sub = this->create_subscription<core::msg::BallPosition>("ball_position", 10, std::bind(&car::ball_pos_cb, this, std::placeholders::_1));
    calibrate_sub = this->create_subscription<core::msg::Calibrate>("calibrate", 10, std::bind(&car::calibrate_cb, this, std::placeholders::_1));
    pos_pub = this->create_publisher<geometry_msgs::msg::Twist>("position", 10);
    calibrate_vel_pub = this->create_publisher<geometry_msgs::msg::Vector3>("vel", 10);
    motor_pub = this->create_publisher<std_msgs::msg::Int32>("motor", 10);

    lidar_pose_data = std::make_shared<ros_tools::msg::LidarPose>();
    ball_pos = std::make_shared<core::msg::BallPosition>();
    calibrate_msg = std::make_shared<core::msg::Calibrate>();
    calibrate_vel = std::make_shared<geometry_msgs::msg::Vector3>();
    target_pose = std::make_shared<geometry_msgs::msg::Twist>();
    motor_msg = std_msgs::msg::Int(); // 1为关闭90度，2为向后释放球并复位

    RCLCPP_INFO(this->get_logger(), "car init");
}

// 初始化car节点控制流程
void car::car_init() {
    // 创建spin线程
    spin_thread = std::make_shared<std::thread>([this]() {
        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    // 第一个点
    front_target = std::make_shared<target>(0.7, 0.0, 0.0, this);
    RCLCPP_INFO(this->get_logger(), "front target set");
    front_target->drive_to_target();
    RCLCPP_INFO(this->get_logger(), "get front target");
    
    // 动作一：初始拾取
    // 开速度, 低速拾球
    RCLCPP_INFO(this->get_logger(), "low speed catch");
    for(int i = 0; i < 20; i++) {
        target_pose->linear.x = 0.1;
        target_pose->linear.y = 0;
        target_pose->angular.z = 0;
        target_pose->linear.z = 0;
        rate->sleep(); // 20hz
    }

    // 发送传送带关闭命令
    motor_msg.data = 1;
    motor_pub->publish(motor_msg);
    RCLCPP_INFO(this->get_logger(), "motor_msg 1 send");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 释放点
    auto release_target = std::make_shared<target>(this);
    RCLCPP_INFO(this->get_logger(), "release target set");
    release_target->drive_to_target();
    RCLCPP_INFO(this->get_logger(), "get release target");

    // 动作二：逐步释放
    // 开速度, 低速释放
    RCLCPP_INFO(this->get_logger(), "begin release");
    motor_msg.data = 2;
    motor_pub->publish(motor_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    RCLCPP_INFO(this->get_logger(), "release success");

    // 随机捞球循环
    while(rclcpp::ok()) {
        rate->sleep();
    }
}

// 激光雷达数据回调
void car::lidar_pose_cb(const ros_tools::msg::LidarPose::SharedPtr msg) {
    lidar_pose_data = msg;
    x = msg->x;
    y = msg->y;
    yaw = msg->yaw;
}

// 球位置回调
void car::ball_pos_cb(const core::msg::BallPosition::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Find ball position: (%f, %f)", msg->x, msg->y);
    ball_pos = msg;
}

// 校准消息回调
void car::calibrate_cb(core::msg::Calibrate::SharedPtr msg) {
    calibrate_msg = msg;
}