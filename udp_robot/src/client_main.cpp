#include "udp_robot/udp_client.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    if(argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <linear_vel_mm_s> <angular_vel_mrad_s>\n";
        return 1;
    }

    rclcpp::init(argc, argv);
    auto client = std::make_shared<UDPClient>();
    
    try {
        int16_t linear = std::stoi(argv[1]);
        int16_t angular = std::stoi(argv[2]);
        
        client->send_message(linear, angular);
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }
    
    rclcpp::shutdown();
    return 0;
}