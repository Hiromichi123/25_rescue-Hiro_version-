#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

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

        // 设置 PointCloud2 的其他参数
        pointcloud2_msg.is_bigendian = false;
        pointcloud2_msg.point_step = 16; // 每个点的字节数
        pointcloud2_msg.row_step = pointcloud2_msg.point_step * custom_msg->point_num;
        pointcloud2_msg.is_dense = true;

        // 分配空间
        pointcloud2_msg.data.resize(pointcloud2_msg.row_step);

        // 填充点云数据
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud->width = custom_msg->point_num;
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        for (size_t i = 0; i < custom_msg->point_num; ++i) {
            cloud->points[i].x = custom_msg->points[i].x;
            cloud->points[i].y = custom_msg->points[i].y;
            cloud->points[i].z = custom_msg->points[i].z;
            cloud->points[i].intensity = custom_msg->points[i].reflectivity;
        }

        // 直通滤波 (保留x, y, z范围内的点)
        pcl::PassThrough<pcl::PointXYZI> pass;
        std::vector<std::tuple<std::string, float, float>> filters = {
            {"x", -1.5, 1.5},
            {"y", -1.5, 1.5},
            {"z", -0.2, 0.0}
        };
        for (const auto& filter : filters) {
            const auto& [axis, min, max] = filter;
            pass.setFilterFieldName(axis);
            pass.setFilterLimits(min, max);
            pass.filter(*cloud);
        }

        // 体素滤波 (降低点云密度)
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.005f, 0.005f, 0.005f);
        voxel_grid.filter(*cloud);

        // 将滤波后的点云数据转换为 PointCloud2 消息
        pcl::toROSMsg(*cloud, pointcloud2_msg);

        // 发布 PointCloud2 消息
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
