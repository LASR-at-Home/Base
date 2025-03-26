cmake_minimum_required(VERSION 3.5)
project(cv2_img)

# Find dependencies
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)  # If you're using OpenCV

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#create-setup-py
# ament_python_install_package(${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
################################################
## Declare ROS messages, services and actions ##
################################################

## In ROS2, message generation is typically done in separate steps.
## If you have custom messages or services, use the rosidl packages:
## Uncomment below if needed.

# find_package(rosidl_default_generators REQUIRED)
# Uncomment if you have messages/services/actions to generate.
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/YourMessage.msg"
#   DEPENDENCIES std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## In ROS2, dynamic reconfigure is replaced with parameters. You can
## configure parameters via YAML or parameter server directly in your nodes.
## No need for dynamic_reconfigure in CMakeLists.

###################################
## ament specific configuration  ##
###################################

## Generate configuration files for dependent packages
ament_package()

###########
## Build ##
###########

## If you have custom C++ code, declare libraries or executables below:

# include_directories(
#   include
#   ${rclpy_INCLUDE_DIRS}
#   ${sensor_msgs_INCLUDE_DIRS}
# )

## If you have a C++ library:
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/cv2_img.cpp
# )

## If you have a C++ executable:
# add_executable(${PROJECT_NAME}_node src/cv2_img_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node
#   ${ament_LIBRARIES}
#   ${rclcpp_LIBRARIES}
#   ${cv_bridge_LIBRARIES}  # If OpenCV is used
# )

#############
## Install ##
#############

# Install Python scripts, if you have any
# install(PROGRAMS
#   scripts/my_python_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files, config, and other files if necessary
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

#############
## Testing ##
#############

## If you have unit tests, add them here. Example for Python:
# ament_add_pytest_test(test_cv2_img test/test_cv2_img.py)
