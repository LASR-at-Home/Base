cmake_minimum_required(VERSION 3.5)
project(open_vocabulary_models)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lasr_vision_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/open_vocabulary_models)

install(PROGRAMS
  src/open_vocabulary_models/node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME open_vocabulary_node
)

install(PROGRAMS
  src/open_vocabulary_models/visualization.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME detection_visualizer
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
