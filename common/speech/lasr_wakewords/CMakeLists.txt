cmake_minimum_required(VERSION 3.5)
project(lasr_wakewords)

# This is a simple Python-node package in ROS2
find_package(ament_cmake REQUIRED)

# Install the Python node as an executable
install(PROGRAMS
  nodes/wakeword_service
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
