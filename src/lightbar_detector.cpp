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
#include<opencv2/calib3d.hpp>
#include"lightbar_detector.h"
#include"config_manager.h"

using namespace cv;
using namespace std;

LightbarDetector::LightbarDetector(const cv::RotatedRect& light)//用于从旋转矩形框中提取参数
{
    width = light.size.width;
    height = light.size.height;
    center = light.center;
    angle = light.angle;
    area = light.size.area();
}

vector<Mat> LightbarDetector::Imagetransform(const Mat& frame)
{
    Mat hsv_image;
    cvtColor(frame,hsv_image,COLOR_BGR2HSV);//转为更符合实际场景的hsv
    vector<Mat> hsvsplit;
    cv::split(hsv_image,hsvsplit);//分离颜色通道
    return hsvsplit;
}

Mat LightbarDetector::Imageprocess(const vector<Mat>& channels)//只接受图片的某个通道（v通道）
{
    Mat frame;
    cv::merge(channels,frame);//通道合并，因为此处颜色操作是对于全视频的
    int threshup=GlobalConfig::getinstance().lightobj.thresh_upper;
    int threshdn=GlobalConfig::getinstance().lightobj.thresh_down;
    //区间一：橙红色
    Scalar lower_red1(0, 80, 80);
    Scalar upper_red1(15, 255, 255);//极优参数

    // 区间 2: 160 - 180 (紫红色)
    Scalar lower_red2(160, 80, 80);
    Scalar upper_red2(180, 255, 255);
    Mat mask1,mask2,red_mask;
    inRange(frame,lower_red1,upper_red1,mask1);//相当于二值化操作
    inRange(frame,lower_red2,upper_red2,mask2);
    red_mask=mask1 | mask2;//已取得较优大致灯条
    //denoise
    blur(red_mask,red_mask,Size(5,5));
    Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)); //膨胀
    dilate(red_mask, red_mask, kernel);


    // threshold(vchannel,Thresh,threshdn,threshup,THRESH_BINARY);
    return red_mask;
}

void LightbarDetector::findcontour(const Mat& frame)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(frame,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
    Mat drawing = Mat::zeros(frame.size(),CV_8UC3);
    for(int i=0;i<contours.size();i++)
    {
        drawContours(drawing,contours,i,cv::Scalar(255,0,0),2,LINE_8,hierarchy,0);
    }
    imshow("contours",drawing);
}