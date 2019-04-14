#include<iostream> 
#include <string> 
#include <ros/ros.h> 
#include <sensor_msgs/JointState.h> 
#include <robot_state_publisher/robot_state_publisher.h> 
#include "achilles_store/pose.h"

using namespace std; 

//读取单片机的状态并发布出来

class achilles_tf
{
    public:
        achilles_tf()
        {
            degree = 3.1415926 / 180;
            joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
            sub_ = nh_.subscribe("/pzt/pose", 1, &achilles_tf::callback, this);

            joint_state.name.resize(5);
            joint_state.position.resize(5);

            joint_state.name[0]="wheel_left_joint";
            joint_state.position[0] = 0;

            joint_state.name[1] ="wheel_right_joint";
            joint_state.position[1] = 0;

            joint_state.name[2] ="front_caster_joint";
            joint_state.position[1] = 0;

            joint_state.name[3] ="servo_1_joint";
            joint_state.name[4] ="servo_2_joint";
        }

        void start()
        {
            while (ros::ok()) 
            { 
                joint_state.header.stamp = ros::Time::now(); 
                joint_pub_.publish(joint_state); 
                loop_rate.sleep();
                ros::spinOnce();
            }
        }

    private:

        void callback(const achilles_store::pose::ConstPtr& msg)
        {
            joint_state.position[3] = msg->angular[0].z;
            joint_state.position[4] = msg->angular[1].y;
        }

        double degree; 

        ros::NodeHandle nh_;
        ros::Publisher joint_pub_;
        ros::Subscriber sub_;
        ros::Rate loop_rate = ros::Rate(10);

        sensor_msgs::JointState joint_state;
};


int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "state_1_publisher");

    achilles_tf a_tf = achilles_tf();

    a_tf.start();

    return 0; 
}
