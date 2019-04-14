#ifndef FlameDetection_common_h
#define FlameDetection_common_h

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>
#include <stack>
#include <queue>
#include <vector>
#include <list>
#include <map>

/*opencv 2.43
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/ml/ml.hpp>
*/
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/video/video.hpp>

//*****************ros所需的头文件***************
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>




using namespace std;
using namespace cv;
using namespace cv::ml;

//#define DEBUG_OUTPUT    //小窗口显示 


#endif
