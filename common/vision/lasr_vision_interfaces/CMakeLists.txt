cmake_minimum_required(VERSION 3.8)
project(lasr_vision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED message_generation message_runtime sensor_msgs)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)  # Needed for built-in ROS types like Time

################################################
## Declare ROS messages, services, and actions #
################################################

# Specify message files
set(msg_files
  "msg/BodyPixKeypoint.msg"
  "msg/BodyPixKeypointNormalized.msg"
  "msg/BodyPixMask.msg"
  "msg/CDRequest.msg"
  "msg/CDResponse.msg"
  "msg/Detection.msg"
  "msg/Detection3D.msg"
)

# Specify service files
set(srv_files
  "srv/BodyPixMaskDetection.srv"
  "srv/BodyPixKeypointDetection.srv"
  "srv/BodyPixWaveDetection.srv"
  "srv/ClipLearnFace.srv"
  "srv/CroppedDetection.srv"
  "srv/LearnFace.srv"
  "srv/YoloDetection.srv"
  "srv/YoloDetection3D.srv"
  "srv/Recognise.srv"
  "srv/DetectFaces.srv"
  "srv/Vqa.srv"
)




# rosidl_generate_interfaces(${PROJECT_NAME}

#   DEPENDENCIES sensor_msgs
# )

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

# Ensure correct build order
ament_package()
