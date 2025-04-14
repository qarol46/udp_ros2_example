#include "udp_math/udp_client.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <num1> <num2>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPClient>();
    node->send_numbers(std::stoi(argv[1]), std::stoi(argv[2]));
    rclcpp::shutdown();
    return 0;
}