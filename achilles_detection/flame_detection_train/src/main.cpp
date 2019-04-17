#include "flame_detection_train/common.h"
#include "flame_detection_train/VideoHandler.h"
#include "flame_detection_train/FlameDetector.h"


#ifdef TRAIN_MODE
bool trainComplete = false;
#endif

VideoHandler* videoHandler = NULL;

int main(int argc, const char* argv[])
{
    //cout << "main : start" << endl;
    if (argc != 2)
    {
        return -1;
    }
    //cout << "main : right argc" << endl;
    
    VideoHandler handler = VideoHandler((string)argv[1], false);
    //cout << "main : VideoHandler handler((std::string)argv[1])\n";

    videoHandler = &handler;
    
    int ret = handler.handle();
    //int ret = 3;
    
    switch (ret) 
    {
        case VideoHandler::STATUS_FLAME_DETECTED:
            //cout << "Flame detected." << endl;
            break;
        case VideoHandler::STATUS_OPEN_CAP_FAILED:
            //cout << "Open capture failed." << endl;
            break;
        case VideoHandler::STATUS_NO_FLAME_DETECTED:
            cout << "No flame detected." << endl;
            break;
        default:
            break;
    }

    return 0;
}
