#include "learning_joy/TeleopJoy.h"
#include "achilles_store/mosSwitch.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <typeinfo>


TeleopJoy::TeleopJoy():
  linear_(1),
  angular_(0)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  nh_.param("Y", increase_linear_, increase_linear_);
  nh_.param("A", decrease_linear_, decrease_linear_);
  nh_.param("X", increase_angular_, increase_linear_);
  nh_.param("B", decrease_angular_, decrease_angular_);
  nh_.param("start_button", start_button_, start_button_);
  nh_.param("stop_button_", stop_button_, stop_button_);
  nh_.param<std::string>("joy_cmd_vel", joy_cmd_vel_, "/cmd_vel");

  //std::cout << joy_cmd_vel_ << std::endl;

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(joy_cmd_vel_, 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);

  client_ = nh_.serviceClient<achilles_store::mosSwitch>("/pzt/switch");

}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if( joy->buttons[increase_linear_] > 0 )
  {
      l_scale_ += 0.1 * l_scale_;
      std::cout <<  "linear : " << l_scale_ << std::endl;
  }
  if( joy->buttons[decrease_linear_] > 0)
  {
      l_scale_ -= 0.1 * l_scale_;
      std::cout <<  "linear : " << l_scale_ << std::endl;
  }

  if( joy->buttons[increase_angular_] > 0)
  {
      a_scale_ += 0.1 * a_scale_;
      std::cout <<  "angular : " << a_scale_ << std::endl;
  }
  if( joy->buttons[decrease_angular_] > 0)
  {
      a_scale_ -= 0.1 * a_scale_;
      std::cout <<  "angular : " << a_scale_ << std::endl;
  }

  if( joy->buttons[start_button_] > 0)
  {
      achilles_store::mosSwitch srv;
      srv.request.needOpen = true;
      if(client_.call(srv))
      {
          std::cout << "服务调用成功" << std::endl;
      }
      else
      {
          std::cout << "服务调用失败" << std::endl;
      }
  }

  if( joy->buttons[stop_button_] > 0)
  {
      achilles_store::mosSwitch srv;
      srv.request.needOpen = false;
      if(client_.call(srv))
      {
          std::cout << "服务调用成功" << std::endl;
      }
      else
      {
          std::cout << "服务调用失败" << std::endl;
      }
  }


  geometry_msgs::Twist twist;
  twist.angular.z =  a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  

  vel_pub_.publish(twist);
}
