#include "udp_robot/udp_client.hpp"
#include "udp_robot/message.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <linear_vel_mm_s> <angular_vel_mrad_s>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPClient>();
    
    Message msg;
    msg.linear_vel = std::stoi(argv[1]);   // Линейная скорость в мм/с
    msg.angular_vel = std::stoi(argv[2]);  // Угловая скорость в мрад/с
    
    node->send_message(msg);
    rclcpp::shutdown();
    return 0;
}