#ifndef TRD_DIFF_CONTROLLER_H
#define TRD_DIFF_CONTROLLER_H

#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_broadcaster.h"

#include "trd_diff_controller/message_manager.h"

#ifndef PI
#define PI 3.141592653
#endif

using namespace std;


class TRDDiffController{
public:
    TRDDiffController();
    TRDDiffController(const char* serial_port_name, int baudrate);
    void cmdVelCallback(const geometry_msgs::Twist &msg);
    void publishOdom();

private:
    ros::NodeHandle nh;
    ros::Publisher pub_imu;
    ros::Publisher pub_odom;
    ros::Subscriber sub_speed;
    sensor_msgs::Imu imu_msg;
    MessageManager message_manager;
    std::string serialport_name;
    int baudrate;
    bool use_imu;
    // base parameters
    double linear_coef;
    double angular_coef;
    double left_coef;
    double right_coef;
    int encoder_ticks_per_rev;
    double wheel_diameter;
    double base_width;
    ros::Time time_current, time_prev;
    int32_t encoder_left_prev, encoder_right_prev;
    double self_x, self_y, self_theta;
    tf::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped tf_transform;
    nav_msgs::Odometry odom;
};

#endif

