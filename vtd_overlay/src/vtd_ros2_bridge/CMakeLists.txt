cmake_minimum_required(VERSION 3.16)
project(vtd_ros2_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_internal_planning_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VtdEgoState.msg"
  "msg/VtdObject.msg"
  "msg/VtdObjectArray.msg"
  "msg/VtdTrafficLight.msg"
  "msg/VtdTrafficLightArray.msg"
  DEPENDENCIES std_msgs
)

set(VTD_ROOT "$ENV{VTD_ROOT}" CACHE PATH "Root of the VTD installation")
find_path(VTD_RDB_INCLUDE_DIR
  NAMES VtdToolkit/viRDBIcd.h
  HINTS
    "${VTD_ROOT}/Develop/Communication/VtdApi/lib_cxx11/include"
    "${VTD_ROOT}/Develop/Communication/VtdFramework/VtdToolkit/include"
)
find_path(VTD_COMMON_INCLUDE_DIR
  NAMES viRDBTypes.h
  HINTS
    "${VTD_ROOT}/Develop/Communication/Common"
)
if(NOT VTD_RDB_INCLUDE_DIR)
  message(FATAL_ERROR
    "VTD RDB headers were not found. Set VTD_ROOT, for example: "
    "export VTD_ROOT=/path/to/VIRES/VTD.2025.2")
endif()
if(NOT VTD_COMMON_INCLUDE_DIR)
  message(FATAL_ERROR
    "VTD common headers were not found: viRDBTypes.h. Set VTD_ROOT to the "
    "root of the VTD installation.")
endif()
message(STATUS "Using VTD RDB headers from ${VTD_RDB_INCLUDE_DIR}")
message(STATUS "Using VTD common headers from ${VTD_COMMON_INCLUDE_DIR}")

add_library(vtd_rdb_core
  src/hlvtd_control_client.cpp
  src/hlvtd_lidar_codec.cpp
  src/rdb_codec.cpp
  src/rdb_shm_reader.cpp
  src/rdb_tcp_client.cpp
  src/rdb_udp_receiver.cpp
)
target_include_directories(vtd_rdb_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE ${VTD_RDB_INCLUDE_DIR} ${VTD_COMMON_INCLUDE_DIR}
)
target_link_libraries(vtd_rdb_core pthread)

add_executable(vtd_bridge_node src/vtd_bridge_node.cpp src/hlvtd_traffic_light_mapping.cpp)
target_include_directories(vtd_bridge_node PRIVATE ${VTD_RDB_INCLUDE_DIR} ${VTD_COMMON_INCLUDE_DIR})
target_link_libraries(vtd_bridge_node vtd_rdb_core)
ament_target_dependencies(vtd_bridge_node
  ament_index_cpp
  autoware_adapi_v1_msgs
  autoware_control_msgs
  autoware_internal_planning_msgs
  autoware_perception_msgs
  autoware_vehicle_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vtd_bridge_node "${cpp_typesupport_target}")

add_executable(vtd_lidar_node src/vtd_lidar_node.cpp)
target_link_libraries(vtd_lidar_node vtd_rdb_core)
ament_target_dependencies(vtd_lidar_node
  rclcpp
  sensor_msgs
)

install(TARGETS vtd_rdb_core vtd_bridge_node vtd_lidar_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/generate_traffic_light_id_map.py
  scripts/generate_hlvtd_traffic_light_id_map.py
  scripts/run_bridge_docker.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hlvtd_control_client test/test_hlvtd_control_client.cpp)
  if(TARGET test_hlvtd_control_client)
    target_link_libraries(test_hlvtd_control_client vtd_rdb_core)
  endif()
  ament_add_gtest(test_hlvtd_lidar_codec test/test_hlvtd_lidar_codec.cpp)
  if(TARGET test_hlvtd_lidar_codec)
    target_link_libraries(test_hlvtd_lidar_codec vtd_rdb_core)
  endif()
  ament_add_gtest(test_rdb_codec test/test_rdb_codec.cpp)
  if(TARGET test_rdb_codec)
    target_include_directories(test_rdb_codec PRIVATE ${VTD_RDB_INCLUDE_DIR} ${VTD_COMMON_INCLUDE_DIR})
    target_link_libraries(test_rdb_codec vtd_rdb_core)
  endif()
  ament_add_gtest(test_rdb_udp_receiver test/test_rdb_udp_receiver.cpp)
  if(TARGET test_rdb_udp_receiver)
    target_include_directories(test_rdb_udp_receiver PRIVATE ${VTD_RDB_INCLUDE_DIR} ${VTD_COMMON_INCLUDE_DIR})
    target_link_libraries(test_rdb_udp_receiver vtd_rdb_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
