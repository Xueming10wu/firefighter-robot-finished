#ifndef __FlameDetection_Train_common_h
#define __FlameDetection_Train_common_h

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




using namespace std;
using namespace cv;
using namespace cv::ml;

//#define OLD_ALGO  //老方法，不必打开

#define TRAIN_MODE    //训练/预测 
//#define DEBUG_OUTPUT    //小窗口显示 

#ifdef TRAIN_MODE
extern bool trainComplete;
#endif

#endif
