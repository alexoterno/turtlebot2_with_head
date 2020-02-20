#!/bin/bash
export ROS_MASTER_URI=http://localhost:11350
export GAZEBO_MASTER_URI=http://localhost:11340
roslaunch launch/my_turtlebot_simulator.launch robot:=_extended gui:=false head_controller:=true pause:=false world_file:=/local_scratch/aauterna/git/src/turtlebot_src/turtlebot_simulator/turtlebot_gazebo/worlds/my_turtlebot_world.world #&
# roslaunch turtlebot_teleop keyboard_teleop.launch
