cmake_minimum_required(VERSION 3.8)
project(udp_robot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(/usr/include)

add_executable(udp_server
  src/udp_server.cpp
  src/server_main.cpp
)
target_include_directories(udp_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(udp_server rclcpp)

add_executable(udp_client
  src/udp_client.cpp
  src/client_main.cpp
)
target_include_directories(udp_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(udp_client rclcpp)

install(TARGETS
  udp_server
  udp_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()