#!/bin/sh
xterm  -e  "roscore" &
xterm  -e  "sleep 2" &&
xterm  -e  "roslaunch usb_cam usb_cam-test.launch" &
xterm  -e  "sleep 5" &&
xterm  -e  "roslaunch ar_track_alvar pr2_indiv_no_kinect.launch" &
xterm  -e  "sleep 2" &&
xterm  -e  "source ~/tello_ws/devel/setup.bash;roslaunch tello_driver tello_node.launch" &
xterm  -e  "source ~/tello_ws/devel/setup.bash;roslaunch tello_control pid_control.launch" &
