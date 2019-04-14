#include "trd_diff_controller/trd_diff_controller.h"

TRDDiffController::TRDDiffController(){
    ros::NodeHandle nh_private("~");
    nh_private.param("use_imu", use_imu, false);
    nh_private.param<std::string>("serialport_name", serialport_name, "/dev/motor_trd");
    nh_private.param("baudrate", baudrate, 38400);
    nh_private.param("linear_coef", linear_coef, 320.0);
    nh_private.param("angular_coef", angular_coef, 60.0);
    nh_private.param("left_coef", left_coef, 1.0);
    nh_private.param("right_coef", right_coef, 1.0);
    nh_private.param("encoder_ticks_per_rev", encoder_ticks_per_rev, 3136); // 49x64
    nh_private.param("wheel_diameter", wheel_diameter, 0.125);
    nh_private.param("base_width", base_width, 0.34);
    if(message_manager.connect(serialport_name.c_str(), baudrate) < 0){
        return;
    }
    message_manager.resetBase();
    usleep(3000000);
    message_manager.setLRCalib();
    message_manager.setTimeout();
    message_manager.resetEncoder();
    //pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 10);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 10);
    sub_speed = nh.subscribe("/cmd_vel", 10, &TRDDiffController::cmdVelCallback, this);
    // ros spin
    ros::Rate loop_rate(10);
    time_prev = ros::Time::now();
    time_current = ros::Time::now();
    encoder_left_prev = 0;
    encoder_right_prev = 0;
    self_x = 0;
    self_y = 0;
    self_theta = 0;
    tf_transform.header.frame_id = "odom";
    tf_transform.child_frame_id = "base_link";
    odom.header.frame_id = "odom";
    odom.child_frame_id= "base_link";
    while(nh.ok()){
        if(use_imu){
            if(message_manager.getEncoderIMU() < 0){
                ROS_WARN("Get encoder_imu failed.");
            }
            else{
                imu_msg.linear_acceleration.x = message_manager.imu_linear_accel_x;
                imu_msg.linear_acceleration.y = message_manager.imu_linear_accel_y;
                imu_msg.linear_acceleration.z = message_manager.imu_linear_accel_z;
                imu_msg.angular_velocity.x = message_manager.imu_angular_vel_x;
                imu_msg.angular_velocity.y = message_manager.imu_angular_vel_y;
                imu_msg.angular_velocity.z = message_manager.imu_angular_vel_z;
                //pub_imu.publish(imu_msg);
                //ROS_INFO("IMU angluar speed x: %f, y: %f, z: %f", \
                //        message_manager.imu_angular_vel_x, message_manager.imu_angular_vel_y, message_manager.imu_angular_vel_z);
                //ROS_INFO("IMU linear accel x: %f, y: %f, z: %f", \
                //        message_manager.imu_linear_accel_x, message_manager.imu_linear_accel_y, message_manager.imu_linear_accel_z);
                //ROS_INFO("IMU orientation x: %f, y: %f, z: %f", \
                //        message_manager.imu_orientation_x, message_manager.imu_orientation_y, message_manager.imu_orientation_z);
            }
        }
        else{
            if(message_manager.getEncoder() < 0){
                ROS_WARN("Get encoder failed.");
            }
            else{
                //ROS_INFO("Got encoder left: %d, right: %d.", int(left_coef*message_manager.encoder_left), int(right_coef*message_manager.encoder_right));
            }
        }
        publishOdom();
        encoder_left_prev = message_manager.encoder_left;
        encoder_right_prev = message_manager.encoder_right;
        
        //ROS_INFO("Got encoder left: %d, right: %d.", int(encoder_left_prev), int(encoder_right_prev));

        ros::spinOnce();
        loop_rate.sleep();
    }
}
void TRDDiffController::cmdVelCallback(const geometry_msgs::Twist &msg){
    // Drive For- or Backward:
    int speed_l = round(linear_coef * msg.linear.x);
    int speed_r = round(linear_coef * msg.linear.x);
    // Turn clock- or counterclockwise:
    speed_l -= round(angular_coef * msg.angular.z);
    speed_r += round(angular_coef * msg.angular.z);        
    speed_l *= left_coef;
    speed_r *= right_coef;
    speed_l	+= 128;
    speed_r += 128;
    if(speed_l>255) speed_l = 255;
    if(speed_l<0)   speed_l = 0;
    if(speed_r>255) speed_r = 255;
    if(speed_r<0)   speed_r = 0; 
    message_manager.setSpeed(speed_l, speed_r);
    //ROS_INFO("Set speed left: %x, right: %x", speed_l, speed_r);
}
void TRDDiffController::publishOdom(){
    double dx, dy, dtheta;
    double d, dleft, dright; 
    time_current = ros::Time::now();
    double elapsed = (time_current - time_prev).toSec();
    time_prev = time_current;
    dleft = left_coef * PI * wheel_diameter * (message_manager.encoder_left - encoder_left_prev) / encoder_ticks_per_rev;
    dright = right_coef * PI * wheel_diameter * (message_manager.encoder_right - encoder_right_prev) / encoder_ticks_per_rev;

    //ROS_INFO("d  left : %f,  right : %f", dleft, dright );
    
    d = (dleft + dright) / 2;
    dtheta = (dright - dleft) / base_width;
    if(d != 0){
        dx = cos(dtheta) * d;
        dy = -sin(dtheta) * d;
        self_x = self_x + ( dx*cos(self_theta) - dy*sin(self_theta) );
        self_y = self_y + ( dx*sin(self_theta) + dy*cos(self_theta) );
    }
    self_theta = self_theta + dtheta;
    // send tf
    geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(self_theta);
    tf_transform.header.stamp = time_current;
    tf_transform.transform.translation.x = self_x;
    tf_transform.transform.translation.y = self_y;
    tf_transform.transform.rotation = odom_quaternion;
    tf_broadcaster.sendTransform(tf_transform);
    // publish odom
    odom.header.stamp = time_current;
    odom.pose.pose.position.x = self_x;
    odom.pose.pose.position.y = self_y;
    odom.pose.pose.orientation = odom_quaternion;
    odom.twist.twist.linear.x = d / elapsed;
    odom.twist.twist.angular.z = dtheta / elapsed;
    pub_odom.publish(odom);
} 
