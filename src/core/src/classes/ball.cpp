#include<car.h>
#include<ball.h>

#define pi 3.14

// 目标点类
// 1.校准点构造函数
target::target(float ball_x, float ball_y, car* car_node) : Node("calibrate_target_node"), reached(false), car_node(car_node), rate(std::make_shared<rclcpp::Rate>(20)) {        
    // 当前位置与球位置之间的向量以及距离
    float dx = car_node->calibrate_msg->global_x;
    float dy = car_node->calibrate_msg->global_x;

    float length = sqrt(dx * dx + dy * dy);
    // 通过单位向量计算target点的位置，距离球0.2m
    float unit_dx = dx / length;
    float unit_dy = dy / length;
    this->x = ball_x - unit_dx * 0.2;
    this->y = ball_y - unit_dy * 0.2;
    // 目标点相对于当前位置的yaw夹角
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

// 非阻塞式移动
void target::drive_to_target(){
    do {car_node->target_pose->linear.x = this->x;
        car_node->target_pose->linear.y = this->y;
        car_node->target_pose->angular.z = this->yaw;
        car_node->target_pose->linear.z = 1; // 标志位
    
        car_node->pos_pub->publish(*car_node->target_pose);
        //TODO：正在移动到点，可以添加突发事件
        rate->sleep();
    } while (!pos_check(car_node->lidar_pose_data));
    car_node->pos_pub->publish(*car_node->target_pose);
}

// 自身位置检查，distance为误差
bool target::pos_check(std::shared_ptr<ros_tools::msg::LidarPose> lidar_pose_data, double distance) {
    return reached || (reached = std::sqrt(std::pow(lidar_pose_data->x - x, 2) +
                                            std::pow(lidar_pose_data->y - y, 2)) < distance &&
                                std::abs(lidar_pose_data->yaw - yaw) < 0.1);
}

//重载严格检查，多误差
bool target::pos_check(std::shared_ptr<ros_tools::msg::LidarPose> lidar_pose_data, double distance_x, double distance_y) {
    return reached || (reached = std::abs(lidar_pose_data->x - x) < distance_x &&
                                std::abs(lidar_pose_data->y - y) < distance_y &&
                                std::abs(lidar_pose_data->yaw - yaw) < 0.1);
}

//--------------------------------------------------------------------------------------------------
// 球类
// 远球构造
ball::ball(float x, float y, car* car_node) : Node("ball_node"), x(x), y(y), car_node(car_node) {
    //pick_release_client = this->create_client<core::srv::PickRelease>("pick_release");
    rate = std::make_shared<rclcpp::Rate>(20);
    // 构造校准点
    calibrate_target = std::make_shared<target>(x, y, car_node);
    calibrate_target->drive_to_target();
    RCLCPP_INFO(this->get_logger(), "get calibrate target");
    calibrate_target = nullptr; // 到达校准点后销毁
}

// 近球构造
ball::ball(car* car_node) : Node("ball_node"), car_node(car_node) {
    //pick_release_client = this->create_client<core::srv::PickRelease>("pick_release");
    rate = std::make_shared<rclcpp::Rate>(20);
}

// 初始化节点控制流程
void ball::ball_init() {
    while(rclcpp::ok()) {
        if (car_node->calibrate_msg->found) {
            // 进入校准模式
            RCLCPP_INFO(this->get_logger(), "enter calibrate mode");
            // 开速度
            car_node->target_pose->linear.x = car_node->lidar_pose_data->x;
            car_node->target_pose->linear.y = car_node->lidar_pose_data->y;
            car_node->target_pose->angular.z = car_node->lidar_pose_data->yaw;
            car_node->target_pose->linear.z = 0; // 标志位
            car_node->pos_pub->publish(*car_node->target_pose);

            while(std::pow(car_node->calibrate_msg->delta_x, 2) + std::pow(car_node->calibrate_msg->delta_y, 2) > 5000) {
                car_node->calibrate_vel->x = car_node->calibrate_msg->delta_x/2000.0;                
                car_node->calibrate_vel->y = car_node->calibrate_msg->delta_y/2000.0;
                car_node->calibrate_vel->z = 0.0;
                car_node->calibrate_vel_pub->publish(*car_node->calibrate_vel);

                /* 
                //位置环
                car_node->target_pose->linear.x = car_node->calibrate_msg->global_x;
                car_node->target_pose->linear.y = car_node->calibrate_msg->global_y;
                car_node->target_pose->angular.z = car_node->lidar_pose_data->yaw;
                car_node->target_pose->linear.z = 1; // 标志位
                car_node->pos_pub->publish(*car_node->target_pose);
                */
                rate->sleep();
            }
            
            // 进入拾取范围
            RCLCPP_INFO(this->get_logger(), "enter pick scope");
            pick_up();
            break;
        }
        rate->sleep();
    }
}

// 拾取动作
void ball::pick_up() {
    // 开速度
    car_node->target_pose->linear.x = car_node->lidar_pose_data->x;
    car_node->target_pose->linear.y = car_node->lidar_pose_data->y;
    car_node->target_pose->angular.z = car_node->lidar_pose_data->yaw;
    car_node->target_pose->linear.z = 0; // 标志位
    car_node->pos_pub->publish(*car_node->target_pose);

    // send_pick_request();
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

    // 位置模式
    car_node->target_pose->linear.x = car_node->lidar_pose_data->x;
    car_node->target_pose->linear.y = car_node->lidar_pose_data->y;
    car_node->target_pose->angular.z = car_node->lidar_pose_data->yaw;
    car_node->target_pose->linear.z = 1; // 标志位

    auto release_target_node = std::make_shared<target>(car_node);
    release_target_node->drive_to_target();

    //send_release_request();
    car_node->motor_msg.data = "release";
    car_node->motor_pub->publish(car_node->motor_msg);
    RCLCPP_INFO(this->get_logger(), "motor_msg send");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(this->get_logger(), "release_request OK");
    release_target_node = nullptr; // 运球结束销毁释放点
}

/*
// 发送拾取请求
void ball::send_pick_request() {
    auto request = std::make_shared<core::srv::PickRelease::Request>();
    request->pick_request = true;
    RCLCPP_INFO(this->get_logger(), "pick_request = true");
    auto result = pick_release_client->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "result wait");
    result.wait();
    RCLCPP_INFO(this->get_logger(), "wait get %d", result.get()->pick_success);
    if (result.get()->pick_success) {
        RCLCPP_INFO(this->get_logger(), "Pick succeeded");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Pick failed");
    }
    request = nullptr;
}

// 发送释放请求
void ball::send_release_request() {
    auto request = std::make_shared<core::srv::PickRelease::Request>();
    request->pick_request = false;
    request->release_request = true;
    auto result = pick_release_client->async_send_request(request);
    result.wait();
    if (result.get()->release_success) {
        RCLCPP_INFO(this->get_logger(), "Release succeeded");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Release failed");
    }
    request = nullptr;
}
*/