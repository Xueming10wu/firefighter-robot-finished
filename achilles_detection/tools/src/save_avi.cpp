//程序功能：实时读取双目视频,分两个窗口分别显示左Camera和右Camera的视频。
//官方网址：莱娜网 www.FpgaLena.com
//程序版本：2016-11-V1.0
#include <iostream>
#include <ctime>
#include <string>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>    
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

const char* keys =
{
	"{help h usage ? | | print this message}"
	"{@video | | Video file, if not defined try to use webcamera}"
};


std::string Get_Current_Date()
{
    time_t nowtime;  
    nowtime = time(NULL); //获取日历时间   
    char tmp[64];   
    strftime(tmp,sizeof(tmp),"%Y-%m-%d_%H:%M:%S",localtime(&nowtime));   
    return tmp;
}

int main(int argc, const char** argv)            //程序主函数
{
	CommandLineParser parser(argc, argv, keys);
	parser.about("Video Capture");

	if (parser.has("help"))                      //帮助信息
	{
		parser.printMessage();
		return 0;
	}
	String videoFile = parser.get<String>(0);

	if (!parser.check())
	{
		parser.printErrors();
		return 0;
	}

	VideoCapture cap;
	if (videoFile != "")
	{
		cap.open(videoFile);
	}
	else
	{
		//--此处需要根据电脑实际连接成像设备的情况，由客户自行进行参数调整------------------------------------
		cap.open(1);                             //打开相机，电脑自带摄像头一般编号为0，外接摄像头编号为1（但不绝对，也可能是反过来的）
	    //--------------------------------------------------------------------------------------

		cap.set(CV_CAP_PROP_FRAME_WIDTH, 2560);  //设置捕获视频的宽度
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);  //设置捕获视频的高度
	}
	if (!cap.isOpened())                         //判断是否成功打开相机
	{ 
		return -1;
	}

    VideoWriter l_writer;
    VideoWriter r_writer;

    string dt = Get_Current_Date();
    string path = "/home/wuxueming/achilles_ws/src/achilles_detection/tools/mp4/";

    string L_file = path + "left/" + dt + ".avi";
    string R_file = path + "right/" + dt + ".avi";

	Mat frame,frame_L,frame_R;
	
	cap >> frame;                                //从相机捕获一帧图像

	Mat grayImage;                               //用于存放灰度数据

	double fScale = 0.5;                         //定义缩放系数，对2560*720图像进行缩放显示（2560*720图像过大，液晶屏分辨率较小时，需要缩放才可完整显示在屏幕）  
	//Size dsize = Size(frame.cols*fScale, frame.rows*fScale);
	Size sumSize = Size( frame.cols, frame.rows );
	Size oneSize = Size( frame.cols * fScale, frame.rows );

	//Mat imagedst = Mat(dsize, CV_32S);
	Mat image = Mat( sumSize, CV_32S );

	//resize(frame, imagedst, dsize);
	resize(frame, image, sumSize);

    int fps = 30;
    //l_writer.open( L_file, CV_FOURCC('M', 'J', 'P', 'G'), fps, dsize );
    //r_writer.open( R_file, CV_FOURCC('M', 'J', 'P', 'G'), fps, dsize );
	l_writer.open( L_file, CV_FOURCC('D', 'I', 'V', 'X'), fps, oneSize );
    r_writer.open( R_file, CV_FOURCC('D', 'I', 'V', 'X'), fps, oneSize );

	while (1)
	{
		cap >> frame;                            //从相机捕获一帧图像
		//resize(frame, imagedst, dsize);          //对捕捉的图像进行缩放操作
		resize(frame, image, sumSize);

		//frame_L = imagedst( Rect(0, 0, 640, 360));  //获取缩放后左Camera的图像
		frame_L = image( Rect(1280, 0, 1280, 720) );  //获取缩放后左Camera的图像
		namedWindow("Video_L", cv::WINDOW_AUTOSIZE); 
	    imshow("Video_L", frame_L);
        l_writer << frame_L;

		//frame_R = imagedst( Rect(640, 0, 640, 360)); //获取缩放后右Camera的图像
		frame_R = image( Rect(0, 0, 1280, 720));
		namedWindow("Video_R", cv::WINDOW_AUTOSIZE); 
		imshow("Video_R", frame_R);
        r_writer << frame_R;

		if (waitKey(30) >= 0) break;
	}

	cap.release();                               //释放对相机的控制
	return 0;
}