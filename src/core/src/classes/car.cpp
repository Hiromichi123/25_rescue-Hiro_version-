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
    //pick_release_service = this->create_service<core::srv::PickRelease>("pick_release", std::bind(&car::handle_pick_release, this,std::placeholders::_1, std::placeholders::_2));
    motor_pub = this->create_publisher<std_msgs::msg::String>("motor", 10); // 添加传送带命令发布者

    lidar_pose_data = std::make_shared<ros_tools::msg::LidarPose>();
    ball_pos = std::make_shared<core::msg::BallPosition>();
    calibrate_msg = std::make_shared<core::msg::Calibrate>();
    calibrate_vel = std::make_shared<geometry_msgs::msg::Vector3>();
    target_pose = std::make_shared<geometry_msgs::msg::Twist>();
    motor_msg = std_msgs::msg::String();

    RCLCPP_INFO(this->get_logger(), "car init");

    // 第一个点
    front_target = std::make_shared<target>(0.7, 0.0, 0.0, this);

    RCLCPP_INFO(this->get_logger(), "front target set");
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
    front_target->drive_to_target();
    RCLCPP_INFO(this->get_logger(), "get front target");
    // 主循环
    while(rclcpp::ok()) {
        if (calibrate_msg->found && ball_thread == nullptr){
            RCLCPP_INFO(this->get_logger(), "set far ball");
            ball_thread = std::make_shared<std::thread>([this]() {
                ball_node = std::make_shared<ball>(calibrate_msg->global_x, calibrate_msg->global_y, this);
                ball_node->pick_up();
                ball_thread.reset();
            });
        }
        /*
        if (ball_pos->found && ball_thread == nullptr) {
            RCLCPP_INFO(this->get_logger(), "into ball");
            ball_thread = std::make_shared<std::thread>([this]() {
                ball_node = std::make_shared<ball>(ball_pos->x, ball_pos->y, this);
                ball_node->ball_init();
            });
        }
        */
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
    if(msg->found){
    RCLCPP_INFO(this->get_logger(), "calibrate g_x: %f, g_y:%f", msg->global_x, msg->global_y);}
    calibrate_msg = msg;
}

/*
// 拾取放置回调
void car::handle_pick_release(const std::shared_ptr<core::srv::PickRelease::Request> out_req, std::shared_ptr<core::srv::PickRelease::Response> response) {
    if (out_req->pick_request) {
        RCLCPP_INFO(this->get_logger(), "Pick up ball");
        motor_msg.data = "pick";
        motor_pub->publish(motor_msg); // 发布“pick”消息
        RCLCPP_INFO(this->get_logger(), "motor_msg send");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        response->pick_success = true;
    }else if(out_req->release_request) {
        RCLCPP_INFO(this->get_logger(), "Release ball");
        motor_msg.data = "release";
        motor_pub->publish(motor_msg); // 发布“release”消息
        response->release_success = true;
        // 销毁ball线程
        if (ball_thread && ball_thread->joinable()) {
            ball_thread->join();
            ball_thread.reset();
            ball_node.reset();
        }
    }else {
        response->pick_success = false;
        response->release_success = false;
    }
}
*/
