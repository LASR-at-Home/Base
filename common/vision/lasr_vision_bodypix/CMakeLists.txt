cmake_minimum_required(VERSION 3.5)
project(lasr_vision_bodypix)

# Find ament packages and libraries for building
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)  # If you are using rclpy in your Python scripts
find_package(sensor_msgs REQUIRED)  # Example ROS2 package dependency
find_package(std_msgs REQUIRED)
find_package(lasr_vision_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

# Install Python scripts (these are the executables)
install(PROGRAMS
  nodes/bodypix_services.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/lasr_vision_bodypix/bodypix.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Macro for generating setup.py
ament_package()
