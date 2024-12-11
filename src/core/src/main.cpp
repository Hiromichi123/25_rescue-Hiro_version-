#include "classes/car.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto car_node = std::make_shared<car>();
    car_node->car_init();
    rclcpp::shutdown();
    return 0;
}

// core代码概括：
// 1.智能指针分治，car类负责大部分共享指针，如服务的一次性指针ball类自己负责，其余同理
// 2.所有对象流程用init封装并对外部暴露接口，兼容一切ros1写法
// 4.target全解耦，三个构造函数以及公有函数提供给除ball外使用
// 5.取消线程锁，ros2内置线程安全
// 7.独立spin线程

// TODO:
// 1.使用行为树代替控制流程