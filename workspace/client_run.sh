#!/bin/bash

# Source the ROS workspace
source /root/workspace/devel/setup.bash

# Start roscore in the background
roscore &

# Wait for roscore to initialize
sleep 5

# Launch the nodes
roslaunch vehicle vehicle.launch &
roslaunch network_sim network_sim.launch &
roslaunch vehicle_sim vehicle_sim.launch 
#roslaunch vehicle_interface vehicle_interface.launch &


# Wait for all background processes
wait
