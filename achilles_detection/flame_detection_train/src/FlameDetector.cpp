//
//  FlameDetector.cpp
//  FlameDetection
//
//  Created by liberize on 14-4-5.
//  Copyright (c) 2014年 liberize. All rights reserved.
//

#include "flame_detection_train/FlameDetector.h"

FlameDetector::FlameDetector()
: mFrameCount(0)
, mFlameCount(0)
, mTrack(false)
{
}

bool FlameDetector::detect(const Mat& frame)
{
    //cout << "start to detect" << endl;

    mFrame = frame;
    
    clock_t start, finish;
    if(++mFrameCount > SKIP_FRAME_COUNT) {
        //cout << "detect : begin if_1" << endl;
        mTrack = true;
        start = clock();
    }
    //cout << "detect : end if_1" << endl;
    
    mExtractor.extract(mFrame, mTargetMap, mTrack);
    //cout << "detect : end extract" << endl;

    if (mTrack) {
        //前几次循环不会执行这里
        //cout << "detect :if (mTrack) is ture" << endl;

        //cout << "detect : analyze(mFrame, mTargetMap) begin" << endl;
        mAnalyzer.analyze(mFrame, mTargetMap);
        //cout << "detect : analyze(mFrame, mTargetMap) end" << endl;


        bool result = mDecider.decide(mFrame, mTargetMap);//故障点
        //cout << "get result\n";

        //cout << "result : " << result << "\n";
        finish = clock();

        //cout << "duration: " << 1.0 * (finish - start) / CLOCKS_PER_SEC << endl;
        if (result) {
            mFlameCount++;
        }
        //cout << "frame: " << (mFrameCount - SKIP_FRAME_COUNT) << ", flame: " << mFlameCount << endl;
        //cout << "detection rate: " << 1.0 * mFlameCount / (mFrameCount - SKIP_FRAME_COUNT) << endl;
        return result;
    }
    
    return false;
}
