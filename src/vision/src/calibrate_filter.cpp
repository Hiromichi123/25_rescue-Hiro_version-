#include "rclcpp/rclcpp.hpp"
#include "core/msg/calibrate.hpp"
#include "std_msgs/msg/string.hpp"
#include <deque>
#include <cmath> // 用于四舍五入

class BallDataFilter : public rclcpp::Node
{
public:
    BallDataFilter() : Node("ball_data_filter")
    {
        // 初始化发布器和订阅器
        publisher_ = this->create_publisher<core::msg::Calibrate>("calibrate_filtered", 10);
        subscription_ = this->create_subscription<core::msg::Calibrate>(
            "calibrate", 10, std::bind(&BallDataFilter::callback, this, std::placeholders::_1));

        // 设置队列大小（5个数据点）
        data_queue_.resize(5);
    }

private:
    void callback(const core::msg::Calibrate::SharedPtr msg)
    {
        // 将接收到的消息放入队列
        data_queue_.push_back(*msg);

        // 如果队列大小超过5，移除最旧的元素
        if (data_queue_.size() > 5) {
            data_queue_.pop_front();
        }

        // 检查队列中是否所有数据的颜色一致
        if (is_color_consistent()) {
            // 如果颜色一致，进行加权滤波
            core::msg::Calibrate filtered_msg = apply_weighted_filter();
            publisher_->publish(filtered_msg);
        }
        else {
            // 如果颜色不一致，发布found = false
            core::msg::Calibrate no_ball_msg;
            no_ball_msg.found = false;
            publisher_->publish(no_ball_msg);
            RCLCPP_INFO(this->get_logger(), "Color inconsistency detected, publishing found=false...");
        }
    }

    bool is_color_consistent()
    {
        // 获取队列中第一个元素的颜色
        int target_color = data_queue_.front().color;
        
        // 检查队列中所有元素的颜色是否一致
        for (const auto& data : data_queue_) {
            if (data.color != target_color) {
                return false; // 如果有不一致的颜色，返回false
            }
        }

        return true; // 所有元素颜色一致，返回true
    }

    core::msg::Calibrate apply_weighted_filter()
    {
        core::msg::Calibrate filtered_msg;
        
        float sum_delta_x = 0.0, sum_delta_y = 0.0;
        float sum_global_x = 0.0, sum_global_y = 0.0;
        int sum_color = data_queue_.front().color; // 颜色保持一致
        bool found = false;

        // 权重分配，最新的数据权重最高，最旧的权重最低
        std::vector<float> weights = {1, 2, 3, 4, 5}; // 假设权重从1到5递增

        float total_weight = 0;
        for (size_t i = 0; i < data_queue_.size(); ++i) {
            float weight = weights[i]; // 为每个数据点分配一个权重

            sum_delta_x += data_queue_[i].delta_x * weight;
            sum_delta_y += data_queue_[i].delta_y * weight;
            sum_global_x += data_queue_[i].global_x * weight;
            sum_global_y += data_queue_[i].global_y * weight;
            found = found || data_queue_[i].found; // If any of the data points indicates 'found', set found to true

            total_weight += weight;
        }

        // 计算加权平均并四舍五入为int
        filtered_msg.delta_x = static_cast<int>(std::round(sum_delta_x / total_weight));
        filtered_msg.delta_y = static_cast<int>(std::round(sum_delta_y / total_weight));
        filtered_msg.global_x = sum_global_x / total_weight;
        filtered_msg.global_y = sum_global_y / total_weight;
        filtered_msg.color = sum_color; // 保持颜色一致
        filtered_msg.found = found;

        RCLCPP_INFO(this->get_logger(),"filtered_msg:delta_x=%d,delta_y=%d,global_x=%f,global_y=%f,color=%d",filtered_msg.delta_x,filtered_msg.delta_y,filtered_msg.global_x,filtered_msg.global_y,filtered_msg.color);

        return filtered_msg;
    }

    rclcpp::Publisher<core::msg::Calibrate>::SharedPtr publisher_;
    rclcpp::Subscription<core::msg::Calibrate>::SharedPtr subscription_;
    std::deque<core::msg::Calibrate> data_queue_; // 队列用于存储最近5个数据
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallDataFilter>());
    rclcpp::shutdown();
    return 0;
}
