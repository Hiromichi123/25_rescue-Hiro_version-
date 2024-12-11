#include "classes/car.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto car_node = std::make_shared<car>();
    car_node->car_init();
    rclcpp::shutdown();
    return 0;
}