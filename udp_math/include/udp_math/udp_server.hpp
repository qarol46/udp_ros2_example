#pragma once

#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <thread>
#include <array>

class UDPServer : public rclcpp::Node {
public:
    UDPServer();
    ~UDPServer();

private:
    void handle_receive(const asio::error_code& error, size_t bytes_transferred);
    void start_receive();

    asio::io_context io_context_;
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint remote_endpoint_;
    std::array<char, 1024> recv_buffer_;
    std::thread io_thread_;
};