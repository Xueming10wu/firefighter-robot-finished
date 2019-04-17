//
//  VideoHandler.cpp
//  FlameDetection
//
//  Created by liberize on 14-4-6.
//  Copyright (c) 2014年 liberize. All rights reserved.
//

#include "flame_detection_train/VideoHandler.h"

VideoHandler::VideoHandler(int device, bool saveKeyFrame, bool saveVideo)
: mCapture(device)
, mSaveKeyFrame(saveKeyFrame)
, mSaveVideo(saveVideo)
, mFromCam(true)
, mVideoFPS(0)
{
    if (mCapture.isOpened()) 
    {   //测试是否能打开摄像头
        //std::cout << "VideoHandler device if()" << endl;
        mVideoFPS = mCapture.get(CV_CAP_PROP_FPS);
        if (mVideoFPS == 0) 
        {
            mVideoFPS = 8.0;
        }
    }
}

VideoHandler::VideoHandler(const string& file, bool saveKeyFrame)
: mCapture(file)
, mSaveKeyFrame(saveKeyFrame)
, mFromCam(false)
, mVideoFPS(0)
{
    //cout << "VideoHandler::VideoHandler\n";
    //mCapture = VideoCapture (file);
    //mSaveKeyFrame = saveKeyFrame;
    //mFromCam = false;
    //mVideoFPS = 0;
    if (mCapture.isOpened()) 
    {   //检测是否正常打开:成功打开时，isOpened返回ture
        
        mVideoFPS = mCapture.get(CV_CAP_PROP_FPS);
        assert(mVideoFPS != 0);
    }
    /*
    if( mSaveVideo  )
    {
        cout << "mSaveVideo is ture" << endl;
    }
    */
}


int VideoHandler::handle()
{
    if (!mCapture.isOpened())
    {   //检测是否正常打开:成功打开时，isOpened返回ture
        return STATUS_OPEN_CAP_FAILED;
    }

    bool continueToDetect = true;
    //能否继续加载分离器

    int extraFrameCount = 0;
    //图片帧数记录

    while (true) 
    {
        //cout << "new true in handler"<< endl;
        if (!mCapture.read(mFrame)) 
        {//读取视频的一帧图片保存到mFrame，失败返回false，成功就返回ture
            //相当于 mCapture >> mFrame 语句带状态返回值
            //cout << (mFromCam ? "Camera disconnected." : "Video file ended.") << endl;
            //如果从摄像头读入失败，则输出 "Camera disconnected."
            //如果从文件读入失败，则输出 "Camera disconnected."
            break;
        }

        
        namedWindow("original", CV_WINDOW_NORMAL);
        //创建一个名为"original"的窗口

        //moveWindow("original", 10, 120);
        ////将显示窗口移到显示屏的相应位置

        imshow("original", mFrame);
        //"original"显示 mFrame，即读取出来的图片

        if (mSaveVideo && !saveVideo()) 
        {   //
            //cout << "Save video failed." << endl;
            mSaveVideo = false;
        }

        //cout << "had show original" <<endl;

        if (continueToDetect) 
        {//如果可以继续进行辨别

            //cout << "in continueToDetect if" << endl;

                      
            if (mDetector.detect(mFrame))               /*故障点*/  
            {//分辨器识别图片
                //cout << "detect(mFrame)" << endl;

                if (mSaveKeyFrame && !saveFrame()) 
                {
                    //cout << "Save key frame failed." << endl;
                }
                if (mSaveVideo)
                {
                    //cout << "in saveVideo" << endl;
                    continueToDetect = false;
                    continue;
                }
                //cout << "Flame detected." << endl;
                //return STATUS_FLAME_DETECTED;
            }
            //cout << "endl continueToDetect if" << endl;
        } 
        else if (++extraFrameCount >= MAX_EXTRA_FRAME_COUNT) 
        {
            //cout << "in continueToDetect else if" << endl;
            return STATUS_FLAME_DETECTED;
        }


        //cout << "pass continueToDetect " << endl;
        //cout << "start #ifdef" << endl;
#ifdef TRAIN_MODE
        if (trainComplete) 
        {
            //cout << "Train complete." << endl;
            break;
        }
#endif
        if (waitKey(WAIT_INTERVAL) == 27) 
        {
            //cout << "User abort." << endl;
            break;
        }

        //cout << "endl #ifdef #endif" << endl << endl << endl;
    }

    return STATUS_NO_FLAME_DETECTED;
}

bool VideoHandler::saveFrame()
{//保存图片文件，并根据时间去命名
    string fileName;
    getCurTime(fileName);
    fileName += ".jpg";
    //cout << "Saving key frame to '" << fileName << "'." << endl;

    return imwrite(fileName, mFrame);
}

bool VideoHandler::saveVideo()
{//保存视频文件
    if (mSaveVideoFile.empty()) 
    {//如果本文件为空，则进行创建
        getCurTime(mSaveVideoFile);
        mSaveVideoFile += ".mov";
        //cout << "Saving video to '" << mSaveVideoFile << "'." << endl;
        
        // in Mac OS X, only 'mp4v' is supported
        int fourcc = CV_FOURCC('m', 'p', '4', 'v');
        Size size = Size((int)mCapture.get(CV_CAP_PROP_FRAME_WIDTH),
                         (int)mCapture.get(CV_CAP_PROP_FRAME_HEIGHT));

        mWriter.open(mSaveVideoFile, fourcc, mVideoFPS, size, true);
    }

    if (!mWriter.isOpened()) 
    {//如果此文件不能打开则 保存失败
        return false;
    }

    mWriter << mFrame;

    return true;
}
