cmake_minimum_required(VERSION 3.0.2)
project(autopilot)

# Path to the dynamicEDT3D package (included in the octomap module)
# For some reason, catkin does not find it with the rest of the packages...
set(dynamicEDT3D_DIR "/opt/ros/melodic/share/dynamicEDT3D/")

# Debug build type
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  octomap_ros
  message_generation
  dynamicEDT3D
  controller_msgs
)

add_message_files(
    FILES
    MotionPrimitive.msg
    BoundingBox.msg
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

set(LOCAL_PLANNER_SOURCE
    src/LocalPlanner.cpp
    src/MotionPrimitive.cpp
    src/MotionPrimitiveLibrary.cpp
    src/SingleAxisTrajectory.cpp
)

set(LOCAL_PLANNER_HEADER
    src/MotionPrimitive.h
    src/MotionPrimitiveLibrary.h
    src/SingleAxisTrajectory.h
    src/Vec3.h
)

add_executable(local_planner ${LOCAL_PLANNER_SOURCE} ${LOCAL_PLANNER_HEADER})
target_link_libraries(local_planner 
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${DYNAMICEDT3D_LIBRARIES}
)