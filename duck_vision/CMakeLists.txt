cmake_minimum_required(VERSION 3.8)
project(duck_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)

# libcamera (Raspberry Pi apt source — `libcamera-dev` 0.5.2). Located via pkg-config
# rather than CMake config because the Bookworm package does not ship a CMake module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera>=0.5)

add_executable(camera_node src/camera_node.cpp)
target_include_directories(camera_node PRIVATE include)
target_link_libraries(camera_node PkgConfig::LIBCAMERA)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  std_msgs
  camera_info_manager
)

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
