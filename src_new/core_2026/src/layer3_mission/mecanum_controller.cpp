#include <array>
#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/twist.hpp>
#include <messages/msg/smart_car_control_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class MecanumControllerNode final : public rclcpp::Node {
public:
    MecanumControllerNode() : rclcpp::Node("mecanum_controller") {
        wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.05);
        wheelbase_m_ = declare_parameter<double>("wheelbase_m", 0.18);
        track_width_m_ = declare_parameter<double>("track_width_m", 0.18);
        max_wheel_rpm_ = declare_parameter<double>("max_wheel_rpm", 300.0);

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 20,
            std::bind(&MecanumControllerNode::on_cmd_vel, this, std::placeholders::_1));

        wheel_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/mecanum/wheel_rpm", 20);

        control_pub_ = create_publisher<messages::msg::SmartCarControlSetpoint>(
            "/smart_car/control_setpoint", 20);

        RCLCPP_INFO(get_logger(),
                    "Mecanum controller started. Mapping: [FL, FR, RL, RR].");
    }

private:
    void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        const double vx = msg->linear.x;
        const double vy = msg->linear.y;
        const double wz = msg->angular.z;

        const double l_plus_w = wheelbase_m_ + track_width_m_;
        const double inv_r = 1.0 / std::max(wheel_radius_m_, 1.0e-6);

        // Standard mecanum inverse kinematics (X forward, Y left, Z up):
        // FL = (vx - vy - (L+W) * wz) / r
        // FR = (vx + vy + (L+W) * wz) / r
        // RL = (vx + vy - (L+W) * wz) / r
        // RR = (vx - vy + (L+W) * wz) / r
        std::array<double, 4> wheel_radps = {
            (vx - vy - l_plus_w * wz) * inv_r,
            (vx + vy + l_plus_w * wz) * inv_r,
            (vx + vy - l_plus_w * wz) * inv_r,
            (vx - vy + l_plus_w * wz) * inv_r,
        };

        std::array<double, 4> wheel_rpm{};
        constexpr double radps_to_rpm = 60.0 / (2.0 * M_PI);
        for (size_t i = 0; i < wheel_radps.size(); ++i) {
            wheel_rpm[i] = std::clamp(wheel_radps[i] * radps_to_rpm, -max_wheel_rpm_, max_wheel_rpm_);
        }

        std_msgs::msg::Float32MultiArray wheel_msg;
        wheel_msg.data = {
            static_cast<float>(wheel_rpm[0]),
            static_cast<float>(wheel_rpm[1]),
            static_cast<float>(wheel_rpm[2]),
            static_cast<float>(wheel_rpm[3]),
        };
        wheel_pub_->publish(wheel_msg);

        messages::msg::SmartCarControlSetpoint control_msg;
        control_msg.mode = messages::msg::SmartCarControlSetpoint::SMART_CAR_MODE_MANUAL;
        control_msg.flags = messages::msg::SmartCarControlSetpoint::SMART_CAR_CONTROL_FLAG_ENABLE;
        control_msg.target_speed_mps = static_cast<float>(std::hypot(vx, vy));
        control_msg.target_curvature = 0.0f;
        control_msg.target_yaw_rate_dps = static_cast<float>(wz * 180.0 / M_PI);
        control_msg.target_accel_mps2 = 0.0f;
        control_pub_->publish(control_msg);
    }

    double wheel_radius_m_{};
    double wheelbase_m_{};
    double track_width_m_{};
    double max_wheel_rpm_{};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_pub_;
    rclcpp::Publisher<messages::msg::SmartCarControlSetpoint>::SharedPtr control_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumControllerNode>());
    rclcpp::shutdown();
    return 0;
}
