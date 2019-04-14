#include "flame_detection/FlameDecider.h"
#include "flame_detection/FlameDetector.h"
#include "flame_detection/FeatureAnalyzer.h"


//const string FlameDecider::SVM_DATA_FILE("svmdata.xml");
extern string SVM_DATA_FILE;

FlameDecider::FlameDecider()
{   
    cout << SVM_DATA_FILE << endl;
    cout << "FlameDecider::svmPredict loading module\n";
    mSVM = Algorithm::load<ml::SVM>(SVM_DATA_FILE.c_str());
    cout << "FlameDecider::svmPredict loading success\n";
}

inline bool FlameDecider::svmPredict(const Feature& feature)
{   
    float result = mSVM->predict(Mat(feature));
	return result == 1.0;
}

bool FlameDecider::judge(map<int, Target>& targets, char LR)
{
    bool flameDetected = false;
    
    Mat temp;
    
    mFrame.copyTo(temp);

    vector<Rect>().swap(resultRegion);
    
    for (map<int, Target>::iterator it = targets.begin(); it != targets.end(); it++) 
    {
        bool isFlame = svmPredict(it->second.feature);

        it->second.isFlame = isFlame;
        if (isFlame) 
        {
            cout << "FlameDecider::judge isFlame \n";
            flameDetected = true;
            rectangle(temp, it->second.region.rect, Scalar(0, 255, 0));

            //把可疑方框放入容器中
            resultRegion.push_back(it->second.region.rect);

            //目标位置 在 second.region.rect 中
            //cout << "x :" <<it->second.region.rect.x <<", y :" << it->second.region.rect.y << endl;
            //cout << "width :" << it->second.region.rect.width << ", height :" << it->second.region.rect.height << endl;
        }
    }
    
    switch (LR)
    {
        case 'L':
            //namedWindow("L_result", CV_WINDOW_NORMAL);
            //imshow("L_result", temp);
            break;
        case 'R':
            //namedWindow("R_result", CV_WINDOW_NORMAL);
            //imshow("R_result", temp);
            break;
        default:
            cout << " mask error" << endl;
    }
    
    //cout << "FlameDecider::judge end\n";

    cv::waitKey(3);
    return flameDetected;
}


bool FlameDecider::decide(const Mat& frame, map<int, Target>& targets, char LR)/*故障点*/
{
    //cout << "decide: start\n";
    frame.copyTo(mFrame);
    
    //cout << "decide: start return judge(targets)\n";
    
    return judge(targets, LR);
   
}

/*
void FlameDecider::getResultRegion( vector<Rect> & in )
*/