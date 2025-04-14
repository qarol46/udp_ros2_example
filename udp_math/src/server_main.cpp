#include "udp_math/udp_server.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}