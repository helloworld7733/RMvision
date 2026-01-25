#include <iostream>
#include<windows.h>
#include<vector>
#include<iomanip>
#include<string>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include<opencv2/calib3d.hpp>

// #include"armor_matcher.h"
#include"config_manager.h"
#include"lightbar_detector.h"
// #include"num_recognizer.h"
// #include"perspective.h"

using namespace std;
using namespace cv;




int main()
{
    VideoCapture cap("D:/University_files/RM/cv/1.3/finalvideo/test01.avi");
    if(!cap.isOpened())
    {
        cerr<<"fail in opening the video"<<endl;
        return -1;
    }
    LightbarDetector obj;
    while(1)
    {
        Mat frame;
        cap>>frame;
        if(frame.empty())
        {
            cout<<"fail in viewing the video"<<endl;
            return -1;
        }

        vector<Mat> hsvsplits=obj.Imagetransform(frame);
        Mat red_mask=obj.Imageprocess(hsvsplits);//传入各个通道，便于后续及额外操作
        vector<LightbarDetector> contours_rect=obj.findcontour(red_mask);
        // imshow("vchannel",red_mask);
        int key=waitKey(10);
        if(key==27)
        {
            break;
        }
        else if(key==32)
        {
            waitKey(0);
        }
    }
    destroyAllWindows();

    return 0;
}
