# tello_control

## install
```
cd catkin_ws/src
git clone https://github.com/lucianobonzatto/tello_control.git
cd ..
catkin_make
```
## dependencies

### codec
```
sudo apt install ros-noetic-codec-image-transport
```

### joy
```
sudo apt-get install ros-noetic-joy
```

### 
```
sudo apt-get install python3-wheel
```

### tellopy
```
cd tello_driver/src/tellopy/tellopy
python3 setup.py bdist_wheel
sudo apt-get install python3-pip
pip3 install dist/tellopy-*.dev*.whl --upgrade
```

## tello_driver
https://github.com/appie-17/tello_driver

connect to tello

### Launch
* Turn on Tello drone
* Connect to drone's WiFi access point (```TELLO_XXXXXX)```
* ```$ roslaunch tello_driver tello_node.launch```

### Subscribed topics
* ```/tello/cmd_vel``` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* ```/tello/emergency``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/fast_mode``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/flattrim``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/flip``` [std_msgs/Uint8](http://docs.ros.org/api/std_msgs/html/msg/UInt8.html)
* ```/tello/land``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/palm_land``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/takeoff``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/manual_takeoff``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/throw_takeoff``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)

### Published topics
* ```/tello/camera/camera_info``` [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
* ```/tello/image_raw``` [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
* ```/tello/imag/raw/h264``` [h264_image_transport/H264Packet](https://github.com/tilk/h264_image_transport/blob/master/msg/H264Packet.msg)
* ```/tello/odom``` [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
* ```/tello/imu``` [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
* ```/tello/status``` [tello_driver/TelloStatus](https://github.com/appie-17/tello_driver/blob/development/msg/TelloStatus.msg)

## camera_info_manager_py
dependencie of tello_driver
https://github.com/ros-perception/camera_info_manager_py

## tello_control
nodes to control the drone

### keyboard control
https://github.com/ros-teleop/teleop_twist_keyboard  
``` ./shell_script/keyboard_control.sh ```
Moving around:  
   i  o  p  
   k  l  ç  
   ,  .  ;   
* anything else: stop  

* u: up (+z)  
* j: down (-z)
* y: takeoff  
* h: land  
  
* q/z: increase/decrease max speeds by 10%  
* w/x: increase/decrease only linear speed by 10%  
* e/c: increase/decrease only angular speed by 10%  

### JOY control
https://github.com/ros-teleop/teleop_twist_joy  
``` ./shell_script/joy_control.sh ```
DS3 control:
* L1: turbo velocity
* L2: normal velocity
* X: takeoff
* square: land
* analog left: x/y
* analog right: z/yal

### hand control
TODO
