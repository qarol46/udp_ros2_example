#pragma once
#include "udp_robot/message.hpp"
#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <thread>

class UDPServer : public rclcpp::Node {
public:
    UDPServer();
    ~UDPServer();

private:
    void calculate_wheel_speeds(const Message& msg, Message& response);
    void handle_receive(const asio::error_code& error, size_t bytes_transferred);
    void start_receive();

    asio::io_context io_context_;
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint remote_endpoint_;
    std::array<uint8_t, sizeof(Message)> recv_buffer_;
    std::thread io_thread_;
    
    const double wheel_separation = 0.8;
    const double wheel_radius = 0.19;
};