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



std::string Get_Current_Date()
{
    time_t nowtime;  
    nowtime = time(NULL); //获取日历时间   
    char tmp[64];   
    strftime(tmp,sizeof(tmp),"%Y-%m-%d_%H:%M:%S",localtime(&nowtime));   
    return tmp;
}


int main(int argc, char *argv[])
{
    cv::VideoWriter writer;

    std::string dt = Get_Current_Date();
    std::string path = "/home/wuxueming/achilles_ws/src/achilles_detection/tools/mp4/";
    

    cv::namedWindow( "orginal", cv::WINDOW_AUTOSIZE );
    //设置窗口

    cv::VideoCapture capture( argv[1] );
    //打开源视频

    cv::Mat bgr_frame;
    //设置图像数组，用来存放capture传过来的图像信息

    double fps = capture.get( cv::CAP_PROP_FPS );
    //获取帧率

    int frames = (int)capture.get( cv::CAP_PROP_FRAME_COUNT );
    //获得总帧数
    cout << "Video has " << frames << " frames of dimensions" << std::endl;

    int current_pos;
    //存放当前帧数

    bool canRead = true; 

    cv::Size size
    (
        (int)capture.get( cv::CAP_PROP_FRAME_WIDTH ),
        (int)capture.get( cv::CAP_PROP_FRAME_HEIGHT )
    );

    int perLength = fps * 5;
    int sumID = frames / perLength;
    //总共 sumID + 1 段视频，长度为 5 秒钟(最后一个不到5秒钟)

    for(int id = 0 ; id <= sumID; id ++ )
    {
        capture >> bgr_frame;
        std::string file = path + "cutted/" + char(id + 48)+ ".avi";
        //文件名字保存，隐式转换

        writer.open( file, CV_FOURCC('D', 'I', 'V', 'X'), fps, size );
        //创建保存视频的文件，设置格式 帧率 尺寸大小

        if (!writer.isOpened()) 
        {//如果此文件不能打开则 保存失败
            std::cout << "writer opened failed" << std::endl;
        }
        else
        {
            std::cout << "writer opened successed" << std::endl;
        }
        
        
        for ( int i = 0 ; i < perLength; i ++)
        {//在一段视频文件中

            capture >> bgr_frame;
            //读取图像

            current_pos = (int)capture.get(cv::CAP_PROP_POS_FRAMES);
            //更新当前帧数
            
            if ( bgr_frame.empty() )
            {//视频结束后更改标志，并退出
                canRead = false;
                break;
            }
            cv::imshow( "orginal", bgr_frame );


            writer << bgr_frame;

            char c = cv::waitKey(10);
            if ( c == 27 )
            {
                canRead = false;
                break;
            }
        }

        //如果超过长度，那就保存视频，并打开下一个保存文件
        writer.release();
        std::cout << "finish one avi" << std::endl;

        if(!canRead)
        {
            break;
        }
    }

    capture.release();
    //关闭视频文件
    
    return 0;
}