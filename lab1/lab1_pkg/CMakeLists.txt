cmake_minimum_required(VERSION 3.8)
project(lab1_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ackermann_msgs REQUIRED)

install(
  PROGRAMS scripts/talker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME talker
)

install(
  PROGRAMS scripts/relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME relay
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
