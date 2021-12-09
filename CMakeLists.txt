cmake_minimum_required(VERSION 3.8)
project(pointcloud_interface_evaluator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(pointcloud_interface_evaluator SHARED
  src/evaluator.cpp
)

target_link_libraries(pointcloud_interface_evaluator
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(pointcloud_interface_evaluator
  PLUGIN "pointcloud_interface_evaluator::Evaluator"
  EXECUTABLE evaluator_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
