#include "udp_robot/udp_server.hpp"
#include <iostream>

UDPServer::UDPServer() 
: Node("udp_server"), 
  socket_(io_context_) {
    
    socket_.open(asio::ip::udp::v4());
    asio::socket_base::reuse_address option(true);
    socket_.set_option(option);
    socket_.bind(asio::ip::udp::endpoint(asio::ip::address::from_string("127.0.0.1"), 12345));
    
    start_receive();
    RCLCPP_INFO(this->get_logger(), "UDP Robot Server started on port 12345");
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
    // Преобразуем линейную и угловую скорости
    double linear = msg.linear_vel;
    double angular = msg.angular_vel;

    // Рассчитываем скорости колес
    double left_speed = (linear - angular * wheel_separation / 2.0) / wheel_radius;
    double right_speed = (linear + angular * wheel_separation / 2.0) / wheel_radius;

    // Заполняем ответ
    response = msg;
    for (int i = 0; i < 3; i++) {
        response.velocity[i] = static_cast<int16_t>(left_speed);
        response.velocity[i+3] = static_cast<int16_t>(right_speed);
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
        Message received_msg;
        std::memcpy(&received_msg, recv_buffer_.data(), sizeof(Message));
        
        // Создаем ответное сообщение с нулевыми массивами
        Message response_msg;
        
        // Копируем заголовок
        response_msg.number_device = received_msg.number_device;
        response_msg.operating_mode = received_msg.operating_mode;
        response_msg.work_device = received_msg.work_device;
        
        // Рассчитываем скорости колес
        double linear = received_msg.linear_vel / 1000.0;
        double angular = received_msg.angular_vel / 1000.0;
        
        double left_speed = (linear - angular * wheel_separation / 2.0) / wheel_radius;
        double right_speed = (linear + angular * wheel_separation / 2.0) / wheel_radius;
        
        // Заполняем скорости колес (левый и правый борт)
        for(int i = 0; i < 3; i++) {
            response_msg.velocity[i] = static_cast<int16_t>(left_speed * 1000);
            response_msg.velocity[i+3] = static_cast<int16_t>(right_speed * 1000);
        }
        
        // Заполняем одометрию (примерные значения)
        for(int i = 0; i < 6; i++) {
            response_msg.odom[i] = static_cast<int16_t>(i * 1000);
        }
        
        // Отправляем ответ
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