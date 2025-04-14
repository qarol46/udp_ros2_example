#pragma once
#include "udp_robot/message.hpp"
#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>

class UDPClient : public rclcpp::Node {
public:
    UDPClient();
    void send_message(const Message& msg);

private:
    void handle_receive(const asio::error_code& error, size_t bytes_transferred);
    void start_receive();

    asio::io_context io_context_;
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint server_endpoint_;
    std::array<uint8_t, sizeof(Message)> recv_buffer_;
    std::atomic<bool> response_received_{false};
};