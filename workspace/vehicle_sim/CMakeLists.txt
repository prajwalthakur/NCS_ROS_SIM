cmake_minimum_required(VERSION 3.0.2)
project(vehicle_sim)

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


add_message_files(
  FILES
  BedPose.msg
  Obstacle.msg
  Perception.msg
  ControlCommand.msg
  ControlCommandSeq.msg
  BedFeedback.msg
  MpcPrediction.msg
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


catkin_install_python(PROGRAMS
  scripts/vehicle_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

