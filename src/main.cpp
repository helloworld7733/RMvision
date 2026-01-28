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

#include"armor_matcher.h"
#include"config_manager.h"
#include"lightbar_detector.h"
#include"num_recognizer.h"
#include"perspective.h"

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
    Mat old_frame;//上一帧
    vector<Armor> old_armors;
    while(1)
    {
        Mat frame;
        cap>>frame;
        Mat resized_frame;
        // if(frame.size()<)
        // resize(frame,resized_frame,Size(1300,1000));
        if(frame.empty())
        {
            cout<<"fail in viewing the video"<<endl;
            return -1;
        }

        vector<Mat> hsvsplits=obj.Imagetransform(frame);
        Mat red_mask=obj.Imageprocess(hsvsplits);//传入各个通道，便于后续及额外操作
        vector<LightbarDetector> contours_rect=obj.findcontour(red_mask);
        Mat drawing=Mat::zeros(red_mask.size(),CV_8UC3);
        for(int i=0;i<contours_rect.size();i++)
        {
            Point2f pts[4];
            contours_rect[i].lightrect.points(pts);
            for(int ii=0;ii<4;ii++)
            {
                line(drawing,pts[ii],pts[(ii+1)%4],cv::Scalar(255,0,0),2,LINE_8);
            }
        }
        ArmorDetector armor_detector;
        vector<Armor> armors=armor_detector.matchbars(contours_rect);
        if(!old_frame.empty()&&armors.empty())//排除首帧
        {
            armor_detector.Frame_tracking(frame,old_frame,old_armors,armors);
        }

        Numrecognizer numrecognizer;
        numrecognizer.Loadsvm("D:/University_files/RM/cv/1.3/final/General/svm.xml");

        Pnpsolver pnpsolver("D:/University_files/RM/cv/1.3/final/General/camera_info.yaml");

        for (auto ele:armors)
        {
            for(int s=0;s<4;s++)
            {
                line(drawing,ele.vertices[s],ele.vertices[(s+1)%4],cv::Scalar(0,0,255));
            }
            circle(drawing,ele.center,3,Scalar(0,255,0),5);
            if(ele.bflag)//是大装甲
            {
                putText(drawing,"Big armor",Point(ele.vertices[1].x,ele.vertices[1].y+2),1,1,cv::Scalar(0,255,0));
            }
            else 
            {
                putText(drawing,"Small armor",Point(ele.vertices[1].x,ele.vertices[1].y+2),1,1,cv::Scalar(0,255,0));
            }
            if(ele.num==0)//防止是旧帧
            {    
                numrecognizer.Loadarmor(ele,frame);
                numrecognizer.num_recognize();
            }
            putText(drawing,to_string(ele.num),Point(ele.vertices[0].x,ele.vertices[0].y-4),1,1,cv::Scalar(0,255,0));
            pnpsolver.Getvec(ele);
            pnpsolver.Practical_info();
            std::stringstream ss;
            ss<<"distance: "<<std::fixed<<std::setprecision(3)<<pnpsolver.distance<<"m";
            putText(drawing,ss.str(),Point(ele.vertices[3].x,ele.vertices[3].y),1,1,cv::Scalar(0,255,0));
        }
        imshow("image",drawing);
        int key=waitKey(10);
        if(key==27)
        {
            break;
        }
        else if(key==32)
        {
            waitKey(0);
        }
        old_frame=frame;
        old_armors=armors;
    }
    destroyAllWindows();

    return 0;
}
