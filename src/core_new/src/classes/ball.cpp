#include<car.h>
#include<ball.h>

#define pi 3.14

// 目标点类
// 1.校准点构造函数
target::target(float ball_x, float ball_y, car* car_node) : Node("calibrate_target_node"), reached(false), car_node(car_node), rate(std::make_shared<rclcpp::Rate>(20)) {        
    float dx = car_node->calibrate_msg->global_x;
    float dy = car_node->calibrate_msg->global_x;
    float length = sqrt(dx * dx + dy * dy);
    float unit_dx = dx / length;
    float unit_dy = dy / length;
    this->x = ball_x - unit_dx * 0.2;
    this->y = ball_y - unit_dy * 0.2;
    float target_angle = atan2(dy, dx);
    this->yaw = target_angle - car_node->lidar_pose_data->yaw;
}

// 2.释放点构造函数
target::target(car* car_node) : Node("release_target_node"), reached(false), car_node(car_node), rate(std::make_shared<rclcpp::Rate>(20)) {
    this->x = 1.0;
    this->y = -0.65;
    this->yaw = pi/2;
}

// 3.一般点构造函数
target::target(float x, float y, float yaw, car* car_node) : Node("simple_target_node"), reached(false), car_node(car_node), rate(std::make_shared<rclcpp::Rate>(20)) {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
}

// 阻塞式移动
void target::drive_to_target(){
    car_node->target_pose->linear.x = this->x;
    car_node->target_pose->linear.y = this->y;
    car_node->target_pose->angular.z = this->yaw;
    car_node->target_pose->linear.z = 1; // 开位置
    do {car_node->pos_pub->publish(*car_node->target_pose);
        rate->sleep();
    } while (!pos_check(car_node->lidar_pose_data, 0.1));
}

// 自身位置检查，distance为误差
bool target::pos_check(std::shared_ptr<ros_tools::msg::LidarPose> lidar_pose_data, double distance) {
    return reached || (reached = std::sqrt(std::pow(lidar_pose_data->x - x, 2) +
                                            std::pow(lidar_pose_data->y - y, 2)) < distance &&
                                std::abs(lidar_pose_data->yaw - yaw) < 0.1);
}

//--------------------------------------------------------------------------------------------------
// 球类
// 远球构造
ball::ball(float x, float y, car* car_node) : Node("ball_node"), x(x), y(y), car_node(car_node) {
    rate = std::make_shared<rclcpp::Rate>(20);
    // 构造校准点
    calibrate_target = std::make_shared<target>(x, y, car_node);
    calibrate_target->drive_to_target();
    calibrate_target = nullptr; // 到达校准点后销毁
}

// 近球构造
ball::ball(car* car_node) : Node("ball_node"), car_node(car_node) {
    rate = std::make_shared<rclcpp::Rate>(20);
}

// 初始化节点控制流
void ball::ball_init() {
    while(rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "enter calibrate mode");
        // 开速度
        car_node->target_pose->linear.x = car_node->lidar_pose_data->x;
        car_node->target_pose->linear.y = car_node->lidar_pose_data->y;
        car_node->target_pose->angular.z = car_node->lidar_pose_data->yaw;
        car_node->target_pose->linear.z = 0;
        
        // 进入拾取范围
        RCLCPP_INFO(this->get_logger(), "enter pick scope");
        pick_up();
        break;
        rate->sleep();
    }
}

// 拾取动作
void ball::pick_up() {
    // 开速度
    car_node->target_pose->linear.x = car_node->lidar_pose_data->x;
    car_node->target_pose->linear.y = car_node->lidar_pose_data->y;
    car_node->target_pose->angular.z = car_node->lidar_pose_data->yaw;
    car_node->target_pose->linear.z = 0;

    car_node->pos_pub->publish(*car_node->target_pose);

    car_node->calibrate_vel->x = 0.25;
    car_node->calibrate_vel->y = 0.0;
    car_node->calibrate_vel_pub->publish(*car_node->calibrate_vel);
    car_node->calibrate_vel->x = 0.25;
    car_node->calibrate_vel->y = 0.0;
    car_node->calibrate_vel_pub->publish(*car_node->calibrate_vel);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    car_node->motor_msg.data = "pick";
    car_node->motor_pub->publish(car_node->motor_msg);
    RCLCPP_INFO(this->get_logger(), "motor_msg send");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    car_node->calibrate_vel->x = 0.0;
    car_node->calibrate_vel->y = 0.0;
    car_node->calibrate_vel_pub->publish(*car_node->calibrate_vel);
    RCLCPP_INFO(this->get_logger(), "ball finish");

    // 开位置
    car_node->target_pose->linear.x = car_node->lidar_pose_data->x;
    car_node->target_pose->linear.y = car_node->lidar_pose_data->y;
    car_node->target_pose->angular.z = car_node->lidar_pose_data->yaw;
    car_node->target_pose->linear.z = 1;

    auto release_target_node = std::make_shared<target>(car_node);
    release_target_node->drive_to_target();

    car_node->motor_msg.data = "release";
    car_node->motor_pub->publish(car_node->motor_msg);
    RCLCPP_INFO(this->get_logger(), "motor_msg send");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(this->get_logger(), "release_request OK");
    release_target_node = nullptr; // 运球结束销毁释放点
}
