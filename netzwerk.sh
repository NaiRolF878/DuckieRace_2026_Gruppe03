#!/bin/bash
source /opt/ros/noetic/setup.bash

export ROS_MASTER_URI=http://donald.local:11311 
#export ROS_MASTER_URI=http://daisy.local:11311 
#export ROS_MASTER_URI=http://tick.local:11311 
#export ROS_MASTER_URI=http://track.local:11311 
#export ROS_MASTER_URI=http://trick.local:11311 
#export ROS_MASTER_URI=http://gustav.local:11311
#export ROS_MASTER_URI=http://dorette.local:11311  
#export ROS_MASTER_URI=http://dagobert.local:11311
#export ROS_MASTER_URI=http://daffy.local:11311  
#export ROS_MASTER_URI=http://gundel.local:11311 

export ROS_IP=192.168.90.187 

export VEHICLE_NAME=donald
#export VEHICLE_NAME=daisy
#export VEHICLE_NAME=tick
#export VEHICLE_NAME=track
#export VEHICLE_NAME=trick
#export VEHICLE_NAME=gustav
#export VEHICLE_NAME=dorette
#export VEHICLE_NAME=dagobert
#export VEHICLE_NAME=daffy
#export VEHICLE_NAME=gundel

cd ~/DuckieRace/

source devel/setup.bash

echo "ROS-Netzwerk geladen! Master:$ROS_MASTER_URI | VM-IP $ROS_IP"
