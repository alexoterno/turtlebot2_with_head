#!/bin/bash
ros_localhost=11350
gazebo_localhost=11450
GUI=false
PAUSE=false
WORLD=my_turtlebot_world
NB_SIM=1
# export ROS_MASTER_URI=http://localhost:11350
# export GAZEBO_MASTER_URI=http://localhost:11450
# roslaunch launch/my_turtlebot_simulator.launch robot:=_extended gui:=false head_controller:=true pause:=false world_file:=/local_scratch/aauterna/git/src/turtlebot_src/turtlebot_simulator/turtlebot_gazebo/worlds/my_turtlebot_world.world &
# # roslaunch turtlebot_teleop keyboard_teleop.launch
# sleep 12s
# export ROS_MASTER_URI=http://localhost:11351
# export GAZEBO_MASTER_URI=http://localhost:11451
# roslaunch launch/my_turtlebot_simulator.launch robot:=_extended gui:=false head_controller:=true pause:=false world_file:=/local_scratch/aauterna/git/src/turtlebot_src/turtlebot_simulator/turtlebot_gazebo/worlds/my_turtlebot_world.world &
# sleep12
# export ROS_MASTER_URI=http://localhost:11352
# export GAZEBO_MASTER_URI=http://localhost:11452
# roslaunch launch/my_turtlebot_simulator.launch robot:=_extended gui:=false head_controller:=true pause:=false world_file:=/local_scratch/aauterna/git/src/turtlebot_src/turtlebot_simulator/turtlebot_gazebo/worlds/my_turtlebot_world.world

for i in "$@"
do
case $i in
    -p|--pause)
    PAUSE=true
    ;;
    -g|--gui)
    GUI=true
    ;;
    -w=*|--world=*)
    WORLD="${i#*=}"
    ;;
    -n=*|--nb_sim=*)
    NB_SIM="${i#*=}"
    ;;
    --default)
    DEFAULT=YES
    ;;
    *)
    echo "Argument error"
    exit 3
    ;;
esac
done
echo PAUSE = ${PAUSE}
echo GUI = ${GUI}
echo WORLD = ${WORLD}
echo NB_SIM = ${NB_SIM}
for i in $(seq 1 1 $((${NB_SIM}-1)))
do
  export ROS_MASTER_URI=http://localhost:$(($ros_localhost+$i-1))
  export  GAZEBO_MASTER_URI=http://localhost:$(($gazebo_localhost+$i-1))
  roslaunch launch/my_turtlebot_simulator.launch robot:=_extended gui:=${GUI} head_controller:=true pause:=${PAUSE} world_file:=/local_scratch/aauterna/git/src/turtlebot_src/turtlebot_simulator/turtlebot_gazebo/worlds/${WORLD}.world &
  sleep 20s
done
export ROS_MASTER_URI=http://localhost:$(($ros_localhost+${NB_SIM}-1))
export GAZEBO_MASTER_URI=http://localhost:$(($gazebo_localhost+${NB_SIM}-1))
roslaunch launch/my_turtlebot_simulator.launch robot:=_extended gui:=${GUI} head_controller:=true pause:=${PAUSE} world_file:=/local_scratch/aauterna/git/src/turtlebot_src/turtlebot_simulator/turtlebot_gazebo/worlds/${WORLD}.world
