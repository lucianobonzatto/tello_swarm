#!/bin/bash

chmod +x multi_tello_connect.sh
chmod +x manager.sh
chmod +x ar_tracker.sh

gnome-terminal --window -- ~/tello_ws/src/tello_control/shell_script/mng_two/multi_tello_connect.sh

sleep 3
gnome-terminal --window -- ~/tello_ws/src/tello_control/shell_script/mng_two/ar_tracker.sh

sleep 3
gnome-terminal --window -- ~/tello_ws/src/tello_control/shell_script/mng_two/manager.sh