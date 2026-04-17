#include "debuger.h"


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Debuger>());
    rclcpp::shutdown();
    return 0;
}