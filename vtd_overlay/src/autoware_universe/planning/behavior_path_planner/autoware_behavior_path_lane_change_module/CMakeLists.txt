cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_path_lane_change_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_path_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/interface.cpp
  src/obstacle_stop_recovery.cpp
  src/manager.cpp
  src/scene.cpp
  src/obstacle_aware.cpp
  src/curvature_braking.cpp
  src/direct_multi_lane.cpp
  src/tactical_lane_selection.cpp
  src/utils/calculation.cpp
  src/utils/markers.cpp
  src/utils/utils.cpp
  src/utils/path.cpp
  src/utils/low_speed.cpp
  src/utils/translated_curve.cpp
)

if(BUILD_TESTING)
  ament_add_gtest(test_translated_curve test/test_translated_curve.cpp)
  target_link_libraries(test_translated_curve ${PROJECT_NAME})
  add_executable(test_corridor_choice test/test_corridor_choice.cpp)
  target_include_directories(test_corridor_choice PRIVATE include)
  target_compile_options(test_corridor_choice PRIVATE -UNDEBUG)
  add_test(NAME test_corridor_choice COMMAND test_corridor_choice)

  ament_add_gtest(test_speed_preparation test/test_speed_preparation.cpp)
  target_link_libraries(test_speed_preparation ${PROJECT_NAME})

  ament_add_gtest(test_obstacle_stop_recovery test/test_obstacle_stop_recovery.cpp)
  target_include_directories(test_obstacle_stop_recovery PRIVATE src)
  target_link_libraries(test_obstacle_stop_recovery ${PROJECT_NAME})

  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_behavior_path_planner_node_interface.cpp
    test/test_lane_change_utils.cpp
    test/test_lane_change_scene.cpp
    test/test_obstacle_aware.cpp
    test/test_planning_factor.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  test_data
)
