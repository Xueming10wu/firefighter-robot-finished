/********************************************************************
*通过订阅双目的目标坐标点与双目视觉的焦距，从而通过视差来确定目标物体的三维坐标， *
*因此此节点 订阅2个话题，发布1个话题。                                    *
*右摄像头为主摄像头。                                                   *
*********************************************************************/
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"

using namespace std;


geometry_msgs::PointStamped left_point;
geometry_msgs::PointStamped right_point;
bool isfire = false;


void leftPiontCB( const geometry_msgs::PointStampedConstPtr& msg )
{
    isfire = true;
    geometry_msgs::PointStamped p = geometry_msgs::PointStamped();
    p.header = msg->header;
    p.point = msg->point;
    //std::cout << "leftPiontCB  x "  << p.point.x <<" ,y  "  << p.point.y << std::endl;
    left_point = p;
}

void rightPiontCB( const geometry_msgs::PointStampedConstPtr& msg )
{
    isfire = true;
    geometry_msgs::PointStamped p = geometry_msgs::PointStamped();
    p.header = msg->header;
    p.point = msg->point;
    //std::cout << "rightPiontCB  x "  << p.point.x <<" ,y  "  << p.point.y << std::endl;
    right_point = p;
}

/*
右侧摄像头
camera matrix
1440.051188 0.000000 768.607899
0.000000 1485.830802 573.545742
0.000000 0.000000 1.000000

左侧摄像头
camera matrix
1310.687953 0.000000 685.837596
0.000000 1311.274435 344.870235
0.000000 0.000000 1.000000



u = fx * x + cx * z
v = fy * y + cy * z

x = (u - cx * z) / fx
y = (v - cy * z) / fy
*/

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "space_node");
    ros::NodeHandle nh;


    //焦距 全部使用平均   上面的内参矩阵 (1,1) (2,2) 求平均
    float left_focus_distance = 1311;
    float right_focus_distance = 1460;

    float left_cx = 666;
    float right_cx = 768;

    float left_cy = 344.870235;
    float right_cy = 573;

    //基线长度   测量部分 单位m
    float base_distance = 0.205;

    //参数
    float Z_a = 0.03273;
    float Z_b = 0.19336;
    float Z_c = -0.00339;

    float X_a = -0.00;
    float X_b = 1;


    //物理坐标系 XYZ 的数值
    float distance_x = 0;
    float distance_y = 0;
    float distance_z = 0;

    ros::Subscriber left_sub = nh.subscribe("/flame/left_2Dposition", 1, leftPiontCB);
    ros::Subscriber right_sub = nh.subscribe("/flame/right_2Dposition", 1, rightPiontCB);
    ros::Publisher space = nh.advertise<geometry_msgs::PointStamped>("/flame/3D_position", 1);
    geometry_msgs::PointStamped msg;
    std::cout << "space_node success" << std::endl;
    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {   
        ros::Duration duration = right_point.header.stamp - left_point.header.stamp;
        double secs;
        secs = duration.toSec();
        //std::cout << " secs = " << secs << std::endl;
        
        if( (ros::Time::now() - left_point.header.stamp).toSec() >= 5)
        {   
            std::cout << "All clear" << std::endl;
            isfire = false;
            nh.setParam("isFire", false);
        }
        if( fabs(secs) < 0.2 && isfire)
        {//只有左右图像相差0.5s内的时候才能够确定实时性
            //双目视觉定位算法获得 物理数据
            //std::cout << " find fire!! " << std::endl;
            nh.setParam("isFire", true);
            float delta =  left_point.point.x - right_point.point.x;

            float d = 1.0;

            float arg_z = left_focus_distance * d /delta;
            //a + bx + cx^2
            distance_z = Z_a + Z_b * arg_z + Z_c * arg_z * arg_z;

            float arg_x = (distance_z / left_focus_distance) * (left_point.point.x - 640);
            //a + bx
            distance_x = X_a + X_b * arg_x;


            distance_y = (distance_z / left_focus_distance) * (left_point.point.y - 360);


            //std::cout << "Z  : " << distance_z << std::endl;
            //std::cout << "X  : " << distance_x << std::endl;
            //std::cout << "Y  : " << distance_y << std::endl;
            //std::cout << "\n\n";
        
            //生成数据帧
            msg.header.frame_id = "left_camera_link";
            msg.header.stamp = ros::Time::now();

            msg.point.x = distance_x;
            msg.point.y = distance_y;
            msg.point.z = distance_z;

            space.publish(msg);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    ros::spin();
    return 0;
}
