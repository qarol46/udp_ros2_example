#pragma once

#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <array>

class UDPClient : public rclcpp::Node {
public:
    UDPClient();
    void send_numbers(int num1, int num2);

private:
    void handle_receive(const asio::error_code& error, size_t bytes_transferred);
    void start_receive();

    asio::io_context io_context_;
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint server_endpoint_;
    std::array<char, 1024> recv_buffer_;
    std::atomic<bool> response_received_{false};
};