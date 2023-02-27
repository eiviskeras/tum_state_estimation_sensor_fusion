cmake_minimum_required(VERSION 3.8)
project(tum_state_estimation_sensor_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(pose_initializer_client src/pose_publisher_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  include
  param
)
