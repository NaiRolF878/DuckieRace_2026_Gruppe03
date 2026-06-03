#!/bin/bash

echo "🚀 Starting DuckieRace ROS environment..."

# ROS Setup
source /opt/ros/noetic/setup.bash

# ROS Master (nur aktiv eine Zeile verwenden!)
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

# WICHTIG: passt die IP ggf. an dein Host-System an
export ROS_IP=192.168.90.187 

# Fahrzeugname (nur aktive eine Zeile verwenden!)
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

# IN Projekt wechseln 
cd ~/DuckieRace/

#Workspace sourcen
if [ -f devel/setup.bash ]; then
    source devel/setup.bash
    echo "✅ ROS workspace geladen"
else
    echo "⚠️ Kein devel/setup.bash gefunden - wurde der Workspace gebaut?"
fi

echo "ROS-Netzwerk geladen!"
echo "ROS-Netzwerk geladen! Master:$ROS_MASTER_URI | VM-IP $ROS_IP"

bash
