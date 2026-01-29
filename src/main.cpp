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
    VideoCapture cap(string(PROJECT_DIR)+"/Video/test01.mp4");//可更换为实际存储路径
    //无法读入异常处理
    if(!cap.isOpened())
    {
        cerr<<"fail in opening the video"<<endl;
        return -1;
    }
    LightbarDetector obj;
    Mat frame,old_frame;//上一帧
    vector<Armor> old_armors;

    Numrecognizer numrecognizer;
    numrecognizer.Loadsvm(string(PROJECT_DIR)+"/General/svm.xml");//可更换为实际存储路径

    Pnpsolver pnpsolver(string(PROJECT_DIR)+"/General/camera_info.yaml");//可更换为实际存储路径

    ArmorDetector armor_detector;

    while(1)
    {
        cap>>frame;
        //无法读入异常处理
        if(frame.empty())
        {
            cout<<"fail in viewing the video"<<endl;
            return -1;
        }
        //灯条识别逻辑
        vector<Mat> hsvsplits=obj.Imagetransform(frame);
        Mat merged_frame=frame.clone();
        Mat mask=obj.Imageprocess(hsvsplits);//传入各个通道，便于后续及额外操作
        vector<LightbarDetector> contours_rect=obj.findcontour(mask);

        Mat drawing=Mat::zeros(mask.size(),CV_8UC3);
        for(int i=0;i<contours_rect.size();i++)
        {
            Point2f pts[4];
            contours_rect[i].lightrect.points(pts);
            for(int ii=0;ii<4;ii++)
            {
                //画出灯条
                line(merged_frame,pts[ii],pts[(ii+1)%4],cv::Scalar(255,0,0),2,LINE_8);
            }
        }
        //装甲板识别逻辑
        vector<Armor> armors=armor_detector.matchbars(contours_rect);

        if(!old_frame.empty()&&armors.empty())//排除首帧
        {
            //跨帧一致性约束
            armor_detector.Frame_tracking(frame,old_frame,old_armors,armors);
        }

        for (auto ele:armors)
        {
            for(int s=0;s<4;s++)
            {
                //画出装甲板你
                line(merged_frame,ele.vertices[s],ele.vertices[(s+1)%4],cv::Scalar(0,0,255));
            }
            //画出装甲板中心
            circle(merged_frame,ele.center,3,Scalar(0,255,0),5);
            if(ele.bflag)//是大装甲
            {
                putText(merged_frame,"Big armor",Point(ele.vertices[1].x,ele.vertices[1].y+2),1,1,cv::Scalar(0,255,0));
            }
            else 
            {
                putText(merged_frame,"Small armor",Point(ele.vertices[1].x,ele.vertices[1].y+2),1,1,cv::Scalar(0,255,0));
            }
            if(ele.num==0)//防止是旧帧
            {    
                //中心数字识别
                numrecognizer.Loadarmor(ele,frame);
                numrecognizer.num_recognize();
            }
            //在左下角显示数字
            putText(merged_frame,to_string(ele.num),Point(ele.vertices[0].x,ele.vertices[0].y-4),1,1,cv::Scalar(0,255,0));
            //Pnp位姿估计，获取实际小车距离信息
            pnpsolver.Getvec(ele);
            pnpsolver.Practical_info();
            std::stringstream ss;
            ss<<"distance: "<<std::fixed<<std::setprecision(3)<<pnpsolver.distance<<"m";
            putText(merged_frame,ss.str(),Point(ele.vertices[3].x,ele.vertices[3].y),1,1,cv::Scalar(0,255,0));
        }
        imshow("image",merged_frame);
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
