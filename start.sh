#!/bin/sh
source /opt/ros/indigo/setup.bash
source ~/pc2oculus/devel/setup.bash 

roslaunch turtlebot_gazebo turtlebot_world.launch &
sleep 5
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 2
roslaunch pc2oculus turtlebot_mapping.launch &
rosnode kill --all
rosnode cleanup
killall -v roslaunch
