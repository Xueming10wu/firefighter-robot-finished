//程序功能：实时读取双目视频,分两个窗口分别显示左Camera和右Camera的视频。
//官方网址：莱娜网 www.FpgaLena.com
//程序版本：2016-11-V1.0

#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>    
#include <opencv2/opencv.hpp> 


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

class LenaCamera
{ 

  VideoCapture cap;
  Mat frame, frame_L, frame_R;
  Size sumSize;
  Mat image;
  
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher L_image_pub_;
  image_transport::Publisher R_image_pub_;
  cv_bridge::CvImage L_cv;
  cv_bridge::CvImage R_cv;

public:
  LenaCamera(std::string dev)
    : it_(nh_)
  {
   
    //声明图像的发布者和订阅者
    L_image_pub_ = it_.advertise("/Lena/left/image_raw", 1);
    R_image_pub_ = it_.advertise("/Lena/right/image_raw", 1);

    cap.open(dev.c_str(), CAP_V4L );                             //打开相机，电脑自带摄像头一般编号为0，外接摄像头编号为1（但不绝对，也可能是反过来的）
    
    //cap.open(openni::ANY_DEVICE);
    //--------------------------------------------------------------------------------------
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 2560);  //设置捕获视频的宽度
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);  //设置捕获视频的高度
    

    if (!cap.isOpened())                         //判断是否成功打开相机
	  { 
        ROS_ERROR("Failed to open camera.\nPlease use \"ls /dev/video*\" in terminal to check your device ");
	  }
    
    cap >> frame;                                //从相机捕获一帧图像
    //定义缩放系数，对2560*720图像进行缩放显示（2560*720图像过大，液晶屏分辨率较小时，需要缩放才可完整显示在屏幕
    sumSize = Size( frame.cols, frame.rows );
    
    image = Mat( sumSize, CV_32S );
    
    publishImage();

    cap.release();                               //释放对相机的控制
  }

  ~LenaCamera()
  {}

  void publishImage()
  {
    while(ros::ok())
    {
        cap >> frame;                            //从相机捕获一帧图像
        resize(frame, image, sumSize);          //对捕捉的图像进行缩放操作


        frame_L = image( Rect(1280, 0, 1280, 720)); //获取缩放后右Camera的图像
        //namedWindow("Video_L", cv::CV_WINDOW_NORMAL); 
	      //imshow("Video_L", frame_L);

        frame_R = image( Rect(0, 0, 1280, 720)); //获取缩放后左Camera的图像
		    //namedWindow("Video_R", cv::CV_WINDOW_NORMAL); 
		    //imshow("Video_R", frame_R);

        ros::Time time=ros::Time::now(); 

        L_cv.header.stamp = time;
        L_cv.header.frame_id = "left_camera_link";
        L_cv.encoding = "bgr8";
        L_cv.image = frame_L;

        R_cv.header.stamp = time;
        R_cv.header.frame_id = "right_camera_link";
        R_cv.encoding = "bgr8";
        R_cv.image = frame_R;
        
        if (waitKey(30) >= 0)
        {
            break;
        }
        std::cout << time << "  Good working" << std::endl;

        sensor_msgs::Image L_image_msg;
        sensor_msgs::Image R_image_msg;

        L_cv.toImageMsg(L_image_msg);
        R_cv.toImageMsg(R_image_msg);

        //将opencv格式数据转换成ros image格式的数据发布
        L_image_pub_.publish(L_image_msg);
        R_image_pub_.publish(R_image_msg); 
    }
    
  }

};

int main(int argc, char** argv)            //程序主函数
{
    ros::init(argc, argv, "lena_node");
    ros::NodeHandle nh;

    std::string dev = "/dev/video0";
    nh.param( "/lena_node/Lena_dev" , dev, dev);

    std::cout << "dev :" << dev << std::endl;

    try
    {
      LenaCamera lenaCamera(dev);
      //打开相机，电脑自带摄像头一般编号为0，外接摄像头编号为1（但不绝对，也可能是反过来的)，这里默认为1
    }
    catch (exception e)
    {
      std::cout << "Error" << std::endl;
      //打开相机，电脑自带摄像头一般编号为0，外接摄像头编号为1（但不绝对，也可能是反过来的)，这里默认为1
      /* code for Catch */
    }
	return 0;
}