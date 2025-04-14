#include "udp_math/udp_server.hpp"
#include <iostream>

UDPServer::UDPServer() 
: Node("udp_server"), 
  socket_(io_context_, asio::ip::udp::endpoint(asio::ip::udp::v4(), 12345)) {
    start_receive();
    RCLCPP_INFO(this->get_logger(), "UDP Server started on port 12345");
    io_thread_ = std::thread([this]() { io_context_.run(); });
}

UDPServer::~UDPServer() {
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    socket_.close();
}

void UDPServer::start_receive() {
    socket_.async_receive_from(
        asio::buffer(recv_buffer_), remote_endpoint_,
        [this](const asio::error_code& error, size_t bytes_transferred) {
            handle_receive(error, bytes_transferred);
        }
    );
}

void UDPServer::handle_receive(const asio::error_code& error, size_t bytes_transferred) {
    if (!error && bytes_transferred == 2 * sizeof(int)) {
        int num1, num2;
        std::memcpy(&num1, recv_buffer_.data(), sizeof(int));
        std::memcpy(&num2, recv_buffer_.data() + sizeof(int), sizeof(int));

        RCLCPP_INFO(this->get_logger(), "Received numbers: %d and %d", num1, num2);

        int results[2] = {num1 + num2, num1 - num2};

        socket_.async_send_to(
            asio::buffer(results), remote_endpoint_,
            [this](const asio::error_code& /*error*/, size_t /*bytes_sent*/) {
                this->start_receive();
            }
        );
    } else {
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Receive error: %s", error.message().c_str());
        }
        start_receive();
    }
}