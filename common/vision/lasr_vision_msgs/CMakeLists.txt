cmake_minimum_required(VERSION 3.5)
project(lasr_vision_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)  # Needed for built-in ROS types like Time

################################################
## Declare ROS messages, services, and actions #
################################################

# Specify message files
set(msg_files
  "msg/BodyPixKeypoint.msg"
  "msg/BodyPixKeypointNormalized.msg"
  "msg/BodyPixMask.msg"
)

# Specify service files
set(srv_files
  "srv/BodyPixMaskDetection.srv"
  "srv/BodyPixKeypointDetection.srv"
  "srv/DetectWave.srv"
)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES sensor_msgs builtin_interfaces  # List all dependencies here
)

###################################
## Export dependencies and package #
###################################

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# This must be called after all targets and before any installation commands
ament_package()
