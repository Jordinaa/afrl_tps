#!/bin/bash



gnome-terminal --tab --active --title="MAVROS" -- bash -c "cd; roslaunch mavros px4.launch; exec bash" Enter
sleep 5
gnome-terminal --tab --active --title="MAVROS" -- bash -c "cd; roslaunch mavros px4.launch; exec bash" Enter
