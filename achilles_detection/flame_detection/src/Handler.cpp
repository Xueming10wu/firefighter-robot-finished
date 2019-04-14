//
//  Handler.cpp
//  FlameDetection
//
//  Created by liberize on 14-4-6.
//  Copyright (c) 2014年 liberize. All rights reserved.
//

#include "flame_detection/Handler.h"


Handler::Handler() 
: it_(nh_)
{
    left_image_sub_ = it_.subscribe("/Lena/left/image_raw", 1, &Handler::leftImageCallBack, this);
    right_image_sub_ = it_.subscribe("/Lena/right/image_raw", 1, &Handler::rightImageCallBack, this);

    left_position_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/flame/left_2Dposition", 1);
    right_position_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/flame/right_2Dposition", 1);
    
    //cv::namedWindow("ros_left_image", CV_WINDOW_NORMAL);
    //cv::namedWindow("ros_right_image", CV_WINDOW_NORMAL);
}


//识别，提供维护性和复用性
bool Handler::get2Dposition(const Mat &img , int & c_x , int & c_y, char LR)
{
    Mat temp;
    c_x = 0;
    c_y = 0;
    
    if (getDetector(LR).detect(img, LR))
    {//如果判断出是火焰的话，我们将黑白前景图片取出，也将可疑区域取出
        
        std::vector<Rect> region = getDetector(LR).getDecider().resultRegion;
        
        temp = img;

        if(region.size() > 0)
        {   //易读性
            c_x = region[0].x + region[0].width/2;
            c_y = region[0].y + region[0].height/2;
            return true;
        }

        /*获取多个火焰目标，图像显示的时候是可以的。但是，为防止目标的不唯一性，不用此部分代码
        for (size_t i = 0; i < region.size(); i++)
        {   
            int c_x = region[i].x + region[i].width/2;
            int c_y = region[i].y + region[i].height/2;
            
            cout << "center: (" << c_x << ", "  << c_y  << ")" << endl;
            //rectangle(temp, region[i], Scalar(0, 255, 0));
            Point p(c_x, c_y);
            circle(temp,p,10,Scalar(255,255,0),3);//第五个参数我们调高点，让线更粗
            
        }
        */
    }
    return false;
}

//****************************ros回调函数**************************************
//*******************************左侧*****************************************
void Handler::leftImageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    //使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //将图片从ros赋值到leftFrame里面
    cv::Mat leftFrame = cv_ptr->image;

    //cv::imshow( "ros_left_image", cv_ptr->image );
    cv::waitKey(3);

    //int 2d_position[2] = {0};
    int x_position = 0;
    int y_position = 0;
    if(get2Dposition(leftFrame , x_position, y_position, 'L'))
    {
        geometry_msgs::PointStamped p;
        p.header.frame_id = "left_camera_link";
        p.header.stamp = ros::Time::now();
        p.point.x = x_position;
        p.point.y = y_position;
        p.point.z = 0;
        left_position_pub_.publish(p);
    }
}

//*******************************右侧*****************************************
void Handler::rightImageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    //使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //将图片从ros赋值到rightFrame里面
    cv::Mat rightFrame = cv_ptr->image;

    //cv::imshow( "ros_right_image", cv_ptr->image );
    cv::waitKey(3);

    //int 2d_position[2] = {0};
    int x_position = 0;
    int y_position = 0;
    if(get2Dposition(rightFrame , x_position, y_position, 'R'))
    {
        geometry_msgs::PointStamped p;
        p.header.frame_id = "right_camera_link";
        p.header.stamp = ros::Time::now();
        p.point.x = x_position;
        p.point.y = y_position;
        p.point.z = 0;
        right_position_pub_.publish(p);
    }
}

