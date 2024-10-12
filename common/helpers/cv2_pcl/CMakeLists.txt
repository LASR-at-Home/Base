cmake_minimum_required(VERSION 3.5)
project(cv2_pcl)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(ros_numpy REQUIRED)  # If you are using ros_numpy for pointcloud conversions
find_package(cv2_img REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed.
## See https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#create-setup-py
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

###################################
## ament specific configuration  ##
###################################

## Generate configuration files for dependent packages
ament_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  # include
  ${rclpy_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

## If you have a C++ library (uncomment if needed)
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/cv2_pcl.cpp
# )

## If you have a C++ executable (uncomment if needed)
# add_executable(${PROJECT_NAME}_node src/cv2_pcl_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node
#   ${ament_LIBRARIES}
#   ${cv_bridge_LIBRARIES}  # Link OpenCV libraries if used
# )

#############
## Install ##
#############

# Install Python scripts (for Python-based nodes)
# install(PROGRAMS
#   scripts/my_python_script.py  # Add your Python script
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install other files (e.g., launch, config files)
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

#############
## Testing ##
#############

## Example of adding tests (for Python-based tests)
# ament_add_pytest_test(test_cv2_pcl test/test_cv2_pcl.py)
