#!/bin/bash
export ROS_MASTER_URI=http://localhost:11351
export GAZEBO_MASTER_URI=http://localhost:11341
roslaunch launch/my_turtlebot_simulator.launch robot:=_extended gui:=false head_controller:=true pause:=false world_file:=/local_scratch/aauterna/git/src/turtlebot_src/turtlebot_simulator/turtlebot_gazebo/worlds/my_turtlebot_world.world
