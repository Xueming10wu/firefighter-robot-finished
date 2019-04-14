#ifndef __FlameDetection__FlameDetector__
#define __FlameDetection__FlameDetector__

#include "flame_detection/common.h"
#include "flame_detection/TargetExtractor.h"
#include "flame_detection/FeatureAnalyzer.h"
#include "flame_detection/FlameDecider.h"


struct Target {
    static const int TARGET_EXISTING = 0;
    static const int TARGET_NEW = 1;
    static const int TARGET_LOST = 2;
    static const int TARGET_MERGED = 3;
    
    int type;
    int times;
    int lostTimes;
    vector<int> mergeSrc;
    Region region;
    Feature feature;
    bool isFlame;
};


class FlameDetector {
private:
    static const int SKIP_FRAME_COUNT = 20;
    
    Mat mFrame;
    TargetExtractor mExtractor;
    FeatureAnalyzer mAnalyzer;
    FlameDecider mDecider;
    map<int, Target> mTargetMap;
    int mFrameCount;
    int mFlameCount;
    bool mTrack;
    
public:
    FlameDetector();
    
    const TargetExtractor& getExtractor() const { return mExtractor; }
    const FeatureAnalyzer& getAnalyzer() const { return mAnalyzer; }
    const FlameDecider& getDecider() const { return mDecider; }
    bool detect(const Mat& frame, char LR);
};

#endif /* defined(__FlameDetection__FlameDetector__) */
