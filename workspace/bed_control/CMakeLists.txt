cmake_minimum_required(VERSION 3.0.2)
project(bed_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  tf2_geometry_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BedPose.msg
  Obstacle.msg
  Perception.msg
  ControlCommand.msg
  ControlCommandSeq.msg
  BedFeedback.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)



catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# catkin_install_python(PROGRAMS
#   scripts/hospital_bed_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

