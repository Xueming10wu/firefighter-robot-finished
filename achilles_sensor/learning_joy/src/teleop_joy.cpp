#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "learning_joy/TeleopJoy.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_controll");
  TeleopJoy teleop_joy;
  std::cout << "Y/A: linear  +/-" << std::endl 
    << "B/X: angular +/-" << std::endl;

  ros::spin();
  return 0;
}