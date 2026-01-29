#include<iostream>
#include<windows.h>
#include<vector>
#include<iomanip>
#include<string>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include<opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include<opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include<opencv2/calib3d.hpp>
#include"lightbar_detector.h"
#include"armor_matcher.h"
#include"num_recognizer.h"

using namespace cv;
using namespace std;
using namespace cv::ml;

Numrecognizer::Numrecognizer()
{
    svm=SVM::create();
    armorimgsz=cv::Size(40,40);//固定参数
    armorroi=Mat();
}

void Numrecognizer::Loadsvm(string path)
{
    svm=SVM::load(path);
    //异常处理
    if(svm.empty())
    {
        cerr<<"fail in loading the svm model"<<endl;
        exit(-1);
    }
}

void Numrecognizer::Loadarmor(Armor& armor, const Mat& frame)
{
    armor_ptr=&armor;

    //在此处执行透视变换已提高识别准度，依赖于armor已识别的四个角点(自适应)
    vector<Point2f> dstpts;//顺序：左下，左上，右上，右下
    //关键一招：由于将数字直接撑满识别框可能导致识别准度不足，故在四周留白，将数字位于中心，能提高识别准度
    dstpts.push_back(Point2f(5,35));
    dstpts.push_back(Point2f(5,5));
    dstpts.push_back(Point2f(35,5));
    dstpts.push_back(Point2f(35,35));

    Mat warpmatrix=getPerspectiveTransform(armor_ptr->vertices.data(),dstpts.data());
    Mat warpsrcimg;
    cvtColor(frame,warpsrcimg,COLOR_BGR2GRAY);
    threshold(warpsrcimg,warpsrcimg,20,255,THRESH_BINARY);
    warpPerspective(warpsrcimg,armorroi,warpmatrix,armorimgsz);
    // imshow("warp",armorroi);
}

void Numrecognizer::num_recognize()
{
    armorroi=armorroi.reshape(1,1);
    armorroi.convertTo(armorroi,CV_32FC1);
    //数字识别
    armor_ptr->num=svm->predict(armorroi);
    // cout<<armor_ptr->num<<endl;
}