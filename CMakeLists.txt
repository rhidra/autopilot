cmake_minimum_required(VERSION 3.0.2)
project(autopilot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  octomap_ros
  message_generation
)

add_message_files(
    FILES
    MotionPrimitive.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS octomap_ros message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(octo_node src/octo_node.cpp)
# target_link_libraries(octo_node ${catkin_LIBRARIES})
# add_dependencies(octo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS global_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
