#include "flame_detection_train/FlameDecider.h"
#include "flame_detection_train/FlameDetector.h"
#include "flame_detection_train/FeatureAnalyzer.h"

//此处使用绝对路径，移植的时候修改地址即可
const string FlameDecider::SVM_DATA_FILE("/home/wuxueming/opencv_ws/src/flame_detection_train/model/svmdata.xml");

#ifdef TRAIN_MODE
//此处使用绝对路径，移植的时候修改地址即可
const string FlameDecider::SAMPLE_FILE("/home/wuxueming/opencv_ws/src/flame_detection_train/data/sample.txt");
#endif

#ifdef TRAIN_MODE
FlameDecider::FlameDecider()
: mSampleEnough(false)
, mFlameCount(0)
, mNonFlameCount(0)
, mFrameCount(0)
{
    Feature feature;
    bool isFlame;
    
    ifstream ifs(SAMPLE_FILE.c_str());
    //从文件中将特征输入到向量中 每一行最后一个数字代表 标签
    while (ifs >> feature >> isFlame) 
    {
        //cout << "mFeatureVec.size() : " << mFeatureVec.size() << endl;
        mFeatureVec.push_back(feature);
        mResultVec.push_back(isFlame);
        if (isFlame) 
        {
            mFlameCount++;
        } else {
            mNonFlameCount++;
        }
    }
    ifs.close();
    
    if (mFlameCount >= MIN_SAMPLE_COUNT && mNonFlameCount >= MIN_SAMPLE_COUNT) {
        mSampleEnough = true;
        //cout << "Flame count: " << mFlameCount << ", non-flame count: " << mNonFlameCount << "." << endl;
    }
}
#else
FlameDecider::FlameDecider()
{
    //mSVM->load(SVM_DATA_FILE.c_str());
    mSVM = Algorithm::load<ml::SVM>(SVM_DATA_FILE.c_str());
}
#endif

#ifdef TRAIN_MODE
void FlameDecider::userInput(const map<int, Target>& targets)
{
    ofstream ofs(SAMPLE_FILE.c_str(), ios::app);
    //cout << "user input" << endl;
    for (map<int, Target>::const_iterator it = targets.begin(); it != targets.end(); it++) 
    {
        if (it->second.lostTimes > 0) {
            continue;
        }
        
        const Feature& feature = it->second.feature;
        const Rectangle& rect = it->second.region.rect;
        
        Mat temp;
        mFrame.copyTo(temp);
        bool flag = true;
        
        while (true) {
            int key = waitKey(200);
            switch (key) {
                case -1:    // no key pressed
                    rectangle(temp, rect, flag ? Scalar(0, 0, 255) : Scalar(0, 255, 0));
                    namedWindow("temp", CV_WINDOW_NORMAL);
                    //moveWindow("temp", 350, 400);
                    imshow("temp", temp);
                    flag = !flag;
                    break;
                case 'y':   // press 'y' to add a positive record to sample
                    ofs << feature << true << endl;
#ifdef DEBUG_OUTPUT
                    //cout << "freq: " << feature.frequency << endl;
                    feature.printAreaVec();
#endif
                    mFeatureVec.push_back(feature);
                    mResultVec.push_back(true);
                    mFlameCount++;
                    goto next;
                case 'n':   // press 'n' to add a negative record to sample
                    ofs << feature << false << endl;
                    mFeatureVec.push_back(feature);
                    mResultVec.push_back(false);
                    mNonFlameCount++;
                    goto next;
                case ' ':   // press SPACE to skip current target
                    goto next;
                case 's':   // press 's' to skip current frame
                    goto end;
                case 27:    // press ESC to stop training and exit program
                    trainComplete = true;
                    goto end;
                case 'o':   // press 'o' to stop input and start studying
                    mSampleEnough = true;
                    goto end;
                default:
                    break;
            }
        }

    next:
        if (mFlameCount >= MIN_SAMPLE_COUNT && mNonFlameCount >= MIN_SAMPLE_COUNT) {
            mSampleEnough = true;
            goto end;
        }
    }
    
end:
    ofs.close();
    //cout << "Flame count: " << mFlameCount << ", non-flame count: " << mNonFlameCount << "." << endl;
}

void FlameDecider::svmStudy()
{
    //cout << "FlameDecider::svmStudy" << endl;
    //assert()的作用:如果条件返回false，则终止程序运行
    assert(mFeatureVec.size() == mResultVec.size());
    
    int size = int(mFeatureVec.size());
    //size行(几个)
    int label_array[size] = {0};
	Mat data(size, Feature::LEN, CV_32FC1);
	
    
	for (int i = 0; i < size; i++) {
		Mat(mFeatureVec[i]).copyTo(data.row(i));

        //cout << "the Mat is :\n";
        //cout << mFeatureVec[i] << endl;
        //cout << "the data.row(i) is :\n";
        //cout << data.row(i) << endl;

        //std::cout << "FlameDecider::svmStudy  : mResultVec[i] = " << mResultVec[i] << std::endl;
        label_array[i] = mResultVec[i] ? 1 : 0;
        //label.at<int>(i, 0) = mResultVec[i] ? 1 : 0;
        //cout << "the label is :\n" << label_array[i] << endl;
	}
    Mat label(size, 1, CV_32SC1, label_array);

    

	//CvSVMParams params;
    //params.svm_type = CvSVM::C_SVC;
    //params.kernel_type = CvSVM::LINEAR;
    //params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
    //mSVM.train(data, label, Mat(), Mat(), params)
	//mSVM.save(SVM_DATA_FILE.c_str());

    //cv::Ptr<cv::ml::SVM> mSVM = cv::ml::SVM::create();
    mSVM = cv::ml::SVM::create();
    //cout << "svm setType" << endl;
    mSVM->setType(cv::ml::SVM::C_SVC);

    //cout << "svm setKernel" << endl;
    mSVM->setKernel(cv::ml::SVM::LINEAR);

    mSVM->setGamma(0.01);
    //精确率过低，则调大；过分严格以至于无法识别稍微有点偏差的数据，则调低。

    mSVM->setC(1.0);
    //惩罚因子

    //cout << "svm setTermCriteria" << endl;
    mSVM->setTermCriteria(cv::TermCriteria(CV_TERMCRIT_ITER, 100, 1e-6));
    
    //cout << "svm train" << endl;
    //mSVM->train(data, ml::ROW_SAMPLE, label);
    //cout << "end train" << endl;

    //bug故障点
    //cout << "Ptr<TrainData>" << endl;
    Ptr<TrainData> traindata = ml::TrainData::create(data, ml::ROW_SAMPLE, label);
    mSVM->train(traindata);
    
    //cout << "svm save " << SVM_DATA_FILE << endl;
    mSVM->save(SVM_DATA_FILE.c_str());
    
}

void FlameDecider::train(const map<int, Target>& targets)
{
    if (!mSampleEnough) 
    {
        if (mFrameCount++ % FRAME_GAP == 0) 
        {
            userInput(targets);
        }
        //cout << "FlameDecider::train: out of userInput"<< endl;
    }
    else 
    {
        svmStudy();
        trainComplete = true;
    }
}
#else
inline bool FlameDecider::svmPredict(const Feature& feature)
{
    //cout << "FlameDecider::svmPredict start\n";
    
    float result = mSVM->predict(Mat(feature));
    //cout << "result: " << result << endl;
	return result == 1.0;
}

bool FlameDecider::judge(map<int, Target>& targets)
{
    bool flameDetected = false;
    
    Mat temp;
    mFrame.copyTo(temp);
    
    for (map<int, Target>::iterator it = targets.begin(); it != targets.end(); it++) {
        bool isFlame = svmPredict(it->second.feature);
        it->second.isFlame = isFlame;
        if (isFlame) {
            flameDetected = true;
            rectangle(temp, it->second.region.rect, Scalar(0, 255, 0));
        }
    }
    
    namedWindow("result");
    //moveWindow("result", 350, 400);
    imshow("result", temp);
    return flameDetected;
}
#endif

bool FlameDecider::decide(const Mat& frame, map<int, Target>& targets)/*故障点*/
{
    //cout << "decide: start\n";
    mFrame = frame;
    
    #ifdef TRAIN_MODE
        //cout << "decide: start train(targets)\n";
        train(targets);
        //cout << "decide: end train(targets) and return false\n";
        return false;
    #else
        //cout << "decide: start return judge(targets)\n";
        return judge(targets);
    #endif
}
