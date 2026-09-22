cmake_minimum_required(VERSION 3.14)
project(carla_aeb_installed_library_tests)

# Compile the upstream + CARLA regression tests against the final installed
# library, so the runtime image itself is checked without rebuilding AEB here.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(autoware_autonomous_emergency_braking REQUIRED)
find_package(autoware_test_utils REQUIRED)
find_package(ament_index_cpp REQUIRED)

if(NOT DEFINED AEB_SOURCE_DIR)
  message(FATAL_ERROR "Pass -DAEB_SOURCE_DIR=<overlay package path>")
endif()
enable_testing()
ament_add_gtest(test_aeb "${AEB_SOURCE_DIR}/test/test.cpp")
target_compile_features(test_aeb PRIVATE cxx_std_17)
ament_target_dependencies(test_aeb
  autoware_autonomous_emergency_braking autoware_test_utils ament_index_cpp)
