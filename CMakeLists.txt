cmake_minimum_required(VERSION 3.0.2)
project(autopilot)

# Path to the dynamicEDT3D package (included in the octomap module)
# For some reason, catkin does not find it with the rest of the packages...
set(dynamicEDT3D_DIR "/opt/ros/melodic/share/dynamicEDT3D/")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  octomap_ros
  message_generation
  dynamicEDT3D
)

add_message_files(
    FILES
    MotionPrimitive.msg
)

add_service_files(
    FILES
    LocalGoal.srv
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
  ${DYNAMICEDT3D_INCLUDE_DIRS}
)

# add_executable(octo_node src/octo_node.cpp)
# target_link_libraries(octo_node ${catkin_LIBRARIES})
# add_dependencies(octo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS global_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(local_planner src/local_planner.cpp)
target_link_libraries(local_planner 
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${DYNAMICEDT3D_LIBRARIES}
)