#include "udp_robot/udp_server.hpp"
#include <iostream>

UDPServer::UDPServer() 
: Node("udp_server"), 
  socket_(io_context_) {
    
    socket_.open(asio::ip::udp::v4());
    asio::socket_base::reuse_address option(true);
    socket_.set_option(option);
    socket_.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), 12345));
    
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

void UDPServer::calculate_wheel_speeds(const Message& msg, Message& response) {
    // Выводим полученные значения
    RCLCPP_INFO(this->get_logger(), "Received values:");
    RCLCPP_INFO(this->get_logger(), "Linear vel: %d mm/s", msg.linear_vel);
    RCLCPP_INFO(this->get_logger(), "Angular vel: %d mrad/s", msg.angular_vel);
    
    double linear = msg.linear_vel / 1000.0;  // Переводим в м/с
    double angular = msg.angular_vel / 1000.0; // Переводим в рад/с

    double left = (linear - angular * wheel_separation/2) / wheel_radius;
    double right = (linear + angular * wheel_separation/2) / wheel_radius;

    // Выводим рассчитанные скорости перед отправкой
    RCLCPP_INFO(this->get_logger(), "Calculated wheel speeds:");
    RCLCPP_INFO(this->get_logger(), "Left wheels: %.2f rad/s (%.2f mm/s)", 
               left, left * wheel_radius * 1000);
    RCLCPP_INFO(this->get_logger(), "Right wheels: %.2f rad/s (%.2f mm/s)", 
               right, right * wheel_radius * 1000);

    for(int i = 0; i < 3; i++) {
        response.velocity[i] = static_cast<int16_t>(left * 1000);  // В мм/с
        response.velocity[i+3] = static_cast<int16_t>(right * 1000);
    }
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
    if (!error && bytes_transferred == sizeof(Message)) {
        Message received_msg, response_msg;
        std::memcpy(&received_msg, recv_buffer_.data(), sizeof(Message));
        
        // Копируем заголовочные поля
        response_msg.number_device = received_msg.number_device;
        response_msg.operating_mode = received_msg.operating_mode;
        response_msg.work_device = received_msg.work_device;
        response_msg.sender_addres = received_msg.sender_addres;
        
        calculate_wheel_speeds(received_msg, response_msg);
        
        // Выводим значения, которые будут отправлены
        RCLCPP_INFO(this->get_logger(), "Sending wheel velocities (mm/s):");
        RCLCPP_INFO(this->get_logger(), "Left: %d, %d, %d",
                   response_msg.velocity[0], 
                   response_msg.velocity[1], 
                   response_msg.velocity[2]);
        RCLCPP_INFO(this->get_logger(), "Right: %d, %d, %d",
                   response_msg.velocity[3],
                   response_msg.velocity[4],
                   response_msg.velocity[5]);
        
        std::array<uint8_t, sizeof(Message)> send_buffer;
        std::memcpy(send_buffer.data(), &response_msg, sizeof(Message));
        
        socket_.async_send_to(
            asio::buffer(send_buffer), remote_endpoint_,
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