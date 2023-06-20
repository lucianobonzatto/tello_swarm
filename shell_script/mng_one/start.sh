#!/bin/bash

gnome-terminal --window -- ~/tello_ws/src/tello_control/shell_script/mng_one/pid_control.sh
sleep 5

gnome-terminal --window -- ~/tello_ws/src/tello_control/shell_script/mng_one/tello_control.sh