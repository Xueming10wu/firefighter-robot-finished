#ifndef __FlameDetection__FlameDecider__
#define __FlameDetection__FlameDecider__

#include "flame_detection/common.h"



struct Target;
class Feature;


class FlameDecider {
private:
    //static const string SVM_DATA_FILE;
    
    
    Mat mFrame;
    //CvSVM mSVM;
    cv::Ptr<cv::ml::SVM> mSVM = cv::ml::SVM::create();
    
    bool svmPredict(const Feature& feature);
    bool judge(map<int, Target>& targets, char LR);

    
public:
    FlameDecider();
    bool decide(const Mat& frame, map<int, Target>& targets, char LR);

    //void getResultRegion( vector<Rect> & in ){in = resultRegion; };

    vector<Rect> resultRegion;
};

#endif /* defined(__FlameDetection__FlameDecider__) */
