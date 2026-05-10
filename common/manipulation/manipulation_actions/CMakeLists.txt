cmake_minimum_required(VERSION 3.5)
project(manipulation_actions)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/manipulation_actions)

install(PROGRAMS
  src/manipulation_actions/node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME manipulation_actions_node
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
