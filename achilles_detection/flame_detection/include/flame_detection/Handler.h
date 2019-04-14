#ifndef __FlameDetection__Handler__
#define __FlameDetection__Handler__

#include "flame_detection/common.h"
#include "flame_detection/utils.h"
#include "flame_detection/FlameDetector.h"


class Handler {
private:
    static const int WAIT_INTERVAL = 30;
    static const int MAX_EXTRA_FRAME_COUNT = 80;

    VideoCapture mCapture;
    //修改
    //FlameDetector mDetector;
    FlameDetector LDetector;
    FlameDetector RDetector;

    
    Mat mFrame;

    //********************ros改造开始*******************
    
    ros::NodeHandle nh_;

    //string input_topic;
    //string output_topic;

    image_transport::ImageTransport it_;

    image_transport::Subscriber left_image_sub_;
    image_transport::Subscriber right_image_sub_;

    ros::Publisher left_position_pub_;
    ros::Publisher right_position_pub_;
    
    //************************************************

public:
    static const int STATUS_FLAME_DETECTED = 0;
    static const int STATUS_OPEN_CAP_FAILED = 1;
    static const int STATUS_NO_FLAME_DETECTED = 2;

    //*******************ros*********************
    Handler();
    void leftImageCallBack(const sensor_msgs::ImageConstPtr& msg);
    void rightImageCallBack(const sensor_msgs::ImageConstPtr& msg);
    //*******************ros*********************

    //如果没有火焰，那就返回flase，2d_position设置为0
    //如果有火焰，那就返回true，2d_position里面包含像素点位置信息
    bool get2Dposition(const Mat &img , int & c_x , int & c_y, char LR);

    FlameDetector& getDetector(char LR)
    { 
        switch (LR)
        {
            case 'L':
                return LDetector; 
            case 'R':
                return RDetector;
            default:
                //默认左眼
                return LDetector;
        }
    }

    
};

#endif /* defined(__FlameDetection__Handler__) */
