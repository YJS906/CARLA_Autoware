cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_path_avoidance_by_lane_change_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_path_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scene.cpp
  src/route_policy.cpp
  src/request_lifecycle.cpp
  src/manager.cpp
  src/interface.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_request_lifecycle test/test_request_lifecycle.cpp)
  target_link_libraries(test_request_lifecycle ${PROJECT_NAME})
  target_include_directories(test_request_lifecycle PRIVATE src)

  ament_add_ros_isolated_gmock(test_speed_preparation_gate test/test_speed_preparation_gate.cpp)
  target_link_libraries(test_speed_preparation_gate ${PROJECT_NAME})
  target_include_directories(test_speed_preparation_gate PRIVATE src)

  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_behavior_path_planner_node_interface.cpp
    test/test_bilateral_selection.cpp
    test/test_approval_handoff.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
endif()

ament_auto_package(INSTALL_TO_SHARE config)
