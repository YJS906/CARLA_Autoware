cmake_minimum_required(VERSION 3.14)
project(autoware_drivable_corridor_recovery)
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_core SHARED src/geometry.cpp src/recovery.cpp)
ament_auto_add_library(${PROJECT_NAME}_node SHARED src/node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::drivable_corridor::RecoveryNode"
  EXECUTABLE drivable_corridor_recovery_node)
ament_auto_package(INSTALL_TO_SHARE config)
