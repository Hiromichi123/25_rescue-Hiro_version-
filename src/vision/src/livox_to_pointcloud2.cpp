#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

class LivoxToPointCloud2 : public rclcpp::Node
{
public:
    LivoxToPointCloud2()
    : Node("livox_to_pointcloud2")
    {
        // 订阅 Livox CustomMsg 类型的点云数据
        subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 10, 
            std::bind(&LivoxToPointCloud2::custom_msg_callback, this, std::placeholders::_1)
        );

        // 发布转换后的 PointCloud2 数据
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/pointcloud2", 10);
    }

private:
    void custom_msg_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr custom_msg)
    {
        // 创建 PointCloud2 消息
        sensor_msgs::msg::PointCloud2 pointcloud2_msg;

        // 填充消息头
        pointcloud2_msg.header.stamp = this->get_clock()->now();
        pointcloud2_msg.header.frame_id = custom_msg->header.frame_id; // 使用 Livox 消息的坐标系

        // 定义 PointCloud2 的字段：x, y, z, intensity
        pointcloud2_msg.fields.resize(4);
        
        pointcloud2_msg.fields[0].name = "x";
        pointcloud2_msg.fields[0].offset = 0;
        pointcloud2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud2_msg.fields[0].count = 1;

        pointcloud2_msg.fields[1].name = "y";
        pointcloud2_msg.fields[1].offset = 4;
        pointcloud2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud2_msg.fields[1].count = 1;

        pointcloud2_msg.fields[2].name = "z";
        pointcloud2_msg.fields[2].offset = 8;
        pointcloud2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud2_msg.fields[2].count = 1;

        pointcloud2_msg.fields[3].name = "intensity";
        pointcloud2_msg.fields[3].offset = 12;
        pointcloud2_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud2_msg.fields[3].count = 1;

        // 设置 PointCloud2 的元数据
        size_t point_count = custom_msg->point_num;
        pointcloud2_msg.width = point_count;
        pointcloud2_msg.height = 1; // 无序点云
        pointcloud2_msg.is_dense = false; // 如果点云包含无效点，设置为 false
        pointcloud2_msg.point_step = 16; // 每个点占用 16 字节 (4x float32)
        pointcloud2_msg.row_step = pointcloud2_msg.point_step * pointcloud2_msg.width;
        pointcloud2_msg.data.resize(pointcloud2_msg.row_step);

        // 填充点云数据
        for (size_t i = 0; i < point_count; ++i)
        {
            float x = custom_msg->points[i].x;
            float y = custom_msg->points[i].y;
            float z = custom_msg->points[i].z;
            float intensity = custom_msg->points[i].reflectivity;

            // 拷贝数据到 PointCloud2 的 data 字段
            memcpy(&pointcloud2_msg.data[i * 16 + 0], &x, sizeof(float));
            memcpy(&pointcloud2_msg.data[i * 16 + 4], &y, sizeof(float));
            memcpy(&pointcloud2_msg.data[i * 16 + 8], &z, sizeof(float));
            memcpy(&pointcloud2_msg.data[i * 16 + 12], &intensity, sizeof(float));
        }

        // 发布转换后的 PointCloud2 消息
        publisher_->publish(pointcloud2_msg);
    }

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxToPointCloud2>());
    rclcpp::shutdown();
    return 0;
}