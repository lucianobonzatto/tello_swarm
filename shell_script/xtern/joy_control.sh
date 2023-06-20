#!/bin/sh
xterm  -e  "source ~/tello_ws/devel/setup.bash;roslaunch tello_control joy_control.launch" &
sleep 2
xterm  -e  "source ~/tello_ws/devel/setup.bash;roslaunch tello_driver tello_node.launch" &
