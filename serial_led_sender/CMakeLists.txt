cmake_minimum_required(VERSION 3.8)
project(serial_led_sender)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# LibSerial via pkg-config (common on Ubuntu)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LibSerial REQUIRED libserial)

include_directories(
  ${LibSerial_INCLUDE_DIRS}
)

add_executable(serial_sender_node src/serial_sender_node.cpp)
ament_target_dependencies(serial_sender_node rclcpp)
target_link_libraries(serial_sender_node ${LibSerial_LIBRARIES})

install(TARGETS serial_sender_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
