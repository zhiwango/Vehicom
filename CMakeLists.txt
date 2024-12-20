cmake_minimum_required(VERSION 3.8)
project(vehicom)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(vehicom_node SHARED
  src/vehicom.cpp
  src/udp_packet.cpp
)

rclcpp_components_register_node(vehicom_node
  PLUGIN "vehicom::VEHICOM"
  EXECUTABLE vehicom
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
