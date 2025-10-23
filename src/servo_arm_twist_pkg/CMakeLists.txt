cmake_minimum_required(VERSION 3.22)
project(servo_arm_twist_pkg)

# C++ Libraries #################################################

# Core C++ library for calculations and collision checking.
# Provides interface used by the component node.
set(SERVO_LIB_NAME servo_arm_twist_lib)

# Pose Tracking
set(POSE_TRACKING pose_tracking)

# Component Nodes (Shared libraries) ############################
set(SERVO_COMPONENT_NODE servo_node)
set(SERVO_CONTROLLER_INPUT servo_controller_input)

# Executable Nodes ##############################################
set(SERVO_NODE_MAIN_NAME servo_node_main)
set(POSE_TRACKING_DEMO_NAME servo_pose_tracking_demo)
set(FAKE_SERVO_CMDS_NAME fake_command_publisher)

#################################################################

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#####################
## Component Nodes ##
#####################

# Add executable for using a controller
add_library(${SERVO_CONTROLLER_INPUT} SHARED src/joystick_twist.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "servo_arm_twist_pkg::JoyToServoPub")

#############
## Install ##
#############

# Install Libraries
install(
  TARGETS
    ${SERVO_CONTROLLER_INPUT}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install Binaries
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
