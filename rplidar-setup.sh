#!/bin/bash 

#Variables
USERNAME="pi"
PI_IP="172.20.10.11"

#Setup ROS
source /opt/ros/noetic/setup.bash
source ~/cg2111a/devel/setup.bash
export ROS_MASTER_URI=http://$1:11311
export ROS_HOSTNAME=$2

#roslaunch rplidar_ros start_nodes_2.launch &
roslaunch rplidar_ros start_nodes.launch &
