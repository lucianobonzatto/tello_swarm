
#include "ros/ros.h"
#include "tello_control/teleop_twist_joy.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "teleop_twist_joy_node");

  ros::NodeHandle nh(""), nh_param("~");
  teleop_twist_joy::TeleopTwistJoy joy_teleop(&nh, &nh_param);

  ros::spin();
}
