#!/bin/sh
xterm  -e  "source ~/tello_ws/devel/setup.bash;roslaunch tello_driver tello_node.launch" &
xterm  -e  "source ~/tello_ws/devel/setup.bash;rosrun tello_control teleop_twist_keyboard.py cmd_vel:=tello/cmd_vel" &
