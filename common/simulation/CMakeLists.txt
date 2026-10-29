cmake_minimum_required(VERSION 3.8)
project(simulation)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

# World plugin that holds grasped objects in the gripper (see src/grasp_assist.cpp)
add_library(grasp_assist SHARED src/grasp_assist.cpp)
target_include_directories(grasp_assist PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(grasp_assist PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(grasp_assist ${GAZEBO_LIBRARIES})
target_compile_features(grasp_assist PRIVATE cxx_std_17)

install(TARGETS grasp_assist LIBRARY DESTINATION lib)
install(DIRECTORY
  launch
  worlds
  maps
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

# Register as pal_gazebo_worlds_private so tiago_gazebo finds our worlds
ament_index_register_resource(packages PACKAGE_NAME pal_gazebo_worlds_private)
install(DIRECTORY worlds DESTINATION share/pal_gazebo_worlds_private)

ament_package()
