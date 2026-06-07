cmake_minimum_required(VERSION 3.8)
project(wut_velma_effort_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tf2_ros REQUIRED)

####################################################################################################
# The library of ros2_control controllers ##########################################################
####################################################################################################
add_library(${PROJECT_NAME} SHARED
  src/wut_velma_effort_controller.cpp
  src/joint_impedance.cpp
  src/wut_velma_model.cpp
  src/self_collision_detector.cpp

  src/cart_torque_controller.cpp
  src/cartesian_impedance.cpp

  src/cartesian_trajectory_generator.cpp

  src/test_chainable_controller.cpp
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen pinocchio::pinocchio)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  visualization_msgs
  geometry_msgs
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  std_msgs
  rcutils
  kdl_parser
  orocos_kdl
  urdf
  tf2_ros
  geometry_msgs
)

# ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   DIRECTORY config launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  std_msgs
  rcutils
  pluginlib
)

# Plugins for ros2_control
install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# package with base classes, path to plugins definition
pluginlib_export_plugin_description_file(
  controller_interface
  plugins.xml
)

ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
)

ament_package()
