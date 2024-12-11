#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class FrontCameraNode : public rclcpp::Node {
public:
    FrontCameraNode() : Node("front_camera_node") {
        camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/front", 1);

        front_camera_.open("/dev/front", cv::CAP_V4L2);
        if (!front_camera_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open camera");
            rclcpp::shutdown();
        }

        front_camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        front_camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 640);
        front_camera_.set(cv::CAP_PROP_FPS, 30);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&FrontCameraNode::timer_callback, this));
    }

private:
    void timer_callback() {
        cv::Mat frame;
        front_camera_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame");
            return;
        }

        cv_bridge::CvImage cv_image;
        cv_image.encoding = "bgr8";
        cv_image.image = frame;

        auto ros_image = cv_image.toImageMsg();
        camera_pub_->publish(*ros_image);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
    cv::VideoCapture front_camera_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontCameraNode>());
    rclcpp::shutdown();
    return 0;
}
