#include "flame_detection/common.h"
#include "flame_detection/Handler.h"
#include "flame_detection/FlameDetector.h"

Handler* PtrHandler = NULL;
string SVM_DATA_FILE;


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "flameDetection");
    ros::NodeHandle nh;

    //ros参数赋值
    string s = "svmdata.xml";
    nh.param<string>( "SVM_DATA_FILE", SVM_DATA_FILE, s );
    
    //SVM_DATA_FILE = "svmdata.xml";

    Handler handler = Handler();
    PtrHandler = &handler;
    
    ros::spin(); 
    return 0;
}
