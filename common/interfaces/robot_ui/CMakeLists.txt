cmake_minimum_required(VERSION 3.8)
project(robot_ui)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Confirm.msg"
)

install(PROGRAMS
  scripts/build
  scripts/dev
  scripts/start
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY robot-interface
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
