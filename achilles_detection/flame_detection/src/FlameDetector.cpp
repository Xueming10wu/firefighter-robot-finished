//
//  FlameDetector.cpp
//  FlameDetection
//
//  Created by liberize on 14-4-5.
//  Copyright (c) 2014年 liberize. All rights reserved.
//

#include "flame_detection/FlameDetector.h"

FlameDetector::FlameDetector()
: mFrameCount(0)
, mFlameCount(0)
, mTrack(false)
{
}

bool FlameDetector::detect(const Mat& frame, char LR)
{
    frame.copyTo(mFrame);
    
    clock_t start, finish;
    if(++mFrameCount > SKIP_FRAME_COUNT) {
        mTrack = true;
        start = clock();
    }
    
    mExtractor.extract(mFrame, mTargetMap, mTrack , LR);
    

    if (mTrack) {
        //前几次循环不会执行这里
        mAnalyzer.analyze(mFrame, mTargetMap, LR);

        bool result = mDecider.decide(mFrame, mTargetMap, LR);

        finish = clock();

        //cout << "duration: " << 1.0 * (finish - start) / CLOCKS_PER_SEC << endl;
        if (result) 
        {
            mFlameCount++;
        }
        //cout << "frame: " << (mFrameCount - SKIP_FRAME_COUNT) << ", flame: " << mFlameCount << endl;
        //cout << "detection rate: " << 1.0 * mFlameCount / (mFrameCount - SKIP_FRAME_COUNT) << endl;
        return result;
    }
    
    return false;
}
