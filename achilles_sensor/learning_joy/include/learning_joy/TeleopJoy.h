#ifndef TELEOP_JOY_H
#define TELEOP_JOY_H

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "achilles_store/mosSwitch.h"



class TeleopJoy
{
public:
  TeleopJoy();
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;
  int linear_, angular_;
  double l_scale_, a_scale_;
  
  int increase_linear_, increase_angular_,
      decrease_linear_, decrease_angular_,
      start_button_,stop_button_;
  
  std::string joy_cmd_vel_;
  ros::ServiceClient client_;
  

  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
};

#endif
