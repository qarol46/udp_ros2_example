#include "udp_robot/udp_client.hpp"
#include <iostream>

UDPClient::UDPClient() 
: Node("udp_client"),
  socket_(io_context_) {
    socket_.open(asio::ip::udp::v4());
    socket_.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), 8889));
    RCLCPP_INFO(this->get_logger(), "Client bound to port: 8889");
    server_endpoint_ = asio::ip::udp::endpoint(
        asio::ip::address::from_string("127.0.0.1"), 8888);
}

void UDPClient::send_message(int16_t linear_vel, int16_t angular_vel) {
    Message msg;
    msg.linear_vel = linear_vel;
    msg.angular_vel = angular_vel;

    std::array<uint8_t, sizeof(Message)> send_buffer;
    std::memcpy(send_buffer.data(), &msg, sizeof(Message));
    
    socket_.async_send_to(
        asio::buffer(send_buffer), server_endpoint_,
        [this](const asio::error_code& error, size_t /*bytes_sent*/) {
            if (!error) this->start_receive();
        }
    );
    
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
    if (!error && bytes_transferred == sizeof(Message)) {
        Message response;
        std::memcpy(&response, recv_buffer_.data(), sizeof(Message));
        
        RCLCPP_INFO(this->get_logger(), "Wheel velocities [L]: %d, %d, %d", 
                   response.velocity[0], response.velocity[1], response.velocity[2]);
        RCLCPP_INFO(this->get_logger(), "Wheel velocities [R]: %d, %d, %d",
                   response.velocity[3], response.velocity[4], response.velocity[5]);
        
        response_received_ = true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Receive error: %s", error.message().c_str());
    }
}