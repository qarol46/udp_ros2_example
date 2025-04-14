#include "udp_math/udp_client.hpp"
#include <iostream>

UDPClient::UDPClient() 
: Node("udp_client"),
  socket_(io_context_) {
    socket_.open(asio::ip::udp::v4());
    server_endpoint_ = asio::ip::udp::endpoint(
        asio::ip::address::from_string("127.0.0.1"), 12345);
}

void UDPClient::send_numbers(int num1, int num2) {
    int numbers[2] = {num1, num2};
    
    socket_.async_send_to(
        asio::buffer(numbers), server_endpoint_,
        [this](const asio::error_code& error, size_t /*bytes_sent*/) {
            if (error) {
                RCLCPP_ERROR(this->get_logger(), "Send error: %s", error.message().c_str());
                return;
            }
            this->start_receive();
        }
    );
    
    // Запускаем обработку событий до получения ответа
    while (rclcpp::ok() && !response_received_) {
        io_context_.run_one_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(shared_from_this());
    }
}

void UDPClient::start_receive() {
    socket_.async_receive_from(
        asio::buffer(recv_buffer_), server_endpoint_,
        [this](const asio::error_code& error, size_t bytes_transferred) {
            this->handle_receive(error, bytes_transferred);
        }
    );
}

void UDPClient::handle_receive(const asio::error_code& error, size_t bytes_transferred) {
    if (!error && bytes_transferred == 2 * sizeof(int)) {
        int results[2];
        std::memcpy(results, recv_buffer_.data(), 2 * sizeof(int));
        
        RCLCPP_INFO(this->get_logger(), 
                   "Received results - Sum: %d, Diff: %d", 
                   results[0], results[1]);
        response_received_ = true;
    } else {
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Receive error: %s", error.message().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid data size: %zu", bytes_transferred);
        }
    }
}