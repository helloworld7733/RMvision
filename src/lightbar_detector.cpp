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

//灯条构造函数，从旋转矩形框中提取参数
LightbarDetector::LightbarDetector(const cv::RotatedRect& light)
{
    width = light.size.width;
    height = light.size.height;
    center = light.center;
    angle = light.angle;
    area = light.size.area();
    lightrect=RotatedRect(center,Size2f(width,height),angle);
}

//函数介绍：旋转椭圆规范化，定义高、宽、中心轴
void LightbarDetector::adjustrec(RotatedRect& elps)
{
    //使得height始终大于width
    if(elps.size.height<elps.size.width)
    {
        swap(elps.size.height,elps.size.width);
        elps.angle+=90;
    }
    if(elps.angle>180) elps.angle-=180;
    if(elps.angle<0) elps.angle+=180;
}

//函数介绍：图像形状变化（全部变成1440*1080，因为这是相机参数）与hsv变换
vector<Mat> LightbarDetector::Imagetransform(Mat& frame)
{
    Mat hsv_image;
    if(frame.size().width!=1440||frame.size().height!=1080)//转成所需大小
    {
        resize(frame,frame,Size(1440,1080));
    }
    cvtColor(frame,hsv_image,COLOR_BGR2HSV);//转为更符合实际场景的hsv
    vector<Mat> hsvsplit;
    cv::split(hsv_image,hsvsplit);//分离颜色通道
    return hsvsplit;
}

//图像处理，依据hsv三通道值筛选灯条
Mat LightbarDetector::Imageprocess(const vector<Mat>& channels)
{
    Mat frame,mask1,mask2,mask;
    cv::merge(channels,frame);//通道合并，因为此处颜色操作是对于全视频的
    //亮度通道阈值化（未用上）
    int threshup=GlobalConfig::getinstance().lightobj.thresh_upper;
    int threshdn=GlobalConfig::getinstance().lightobj.thresh_down;

    string enemy_color=GlobalConfig::getinstance().lightobj.enemy_color;

    if(enemy_color=="red")
    {
        //红色在h色调图中有两个区间，故分为两个区间讨论
        //区间一：橙红色
        Scalar lower_red1(0, 60, 90);
        Scalar upper_red1(16, 255, 255);//可调参数

        // 区间 2: 160 - 180 (紫红色) (可选)
        Scalar lower_red2(180, 255, 255);
        Scalar upper_red2(180, 255, 255);

        inRange(frame,lower_red1,upper_red1,mask1);//相当于二值化操作
        inRange(frame,lower_red2,upper_red2,mask2);
        mask=mask1 | mask2;//已取得较优大致灯条
    }
    else //enemy_color="blue"
    {
        Scalar lower_blue(110,80,60);
        Scalar upper_blue(130,255,255);

        inRange(frame,lower_blue,upper_blue,mask1);

        mask=mask1;//蓝色只有一个色调区间
    }

    //denoise
    blur(mask,mask,Size(5,5));
    Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)); //膨胀
    dilate(mask, mask, kernel);

    // threshold(vchannel,Thresh,threshdn,threshup,THRESH_BINARY);
    return mask;
}

//函数介绍：寻找灯条轮廓
vector<LightbarDetector> LightbarDetector::findcontour(const Mat& frame)
{
    vector<vector<Point>> contours;
    vector<LightbarDetector> rect;
    vector<Vec4i> hierarchy;
    findContours(frame,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
    Mat drawing = Mat::zeros(frame.size(),CV_8UC3);
    for(int i=0;i<contours.size();i++)
    {
        //筛选轮廓
        float contourarea=contourArea(contours[i]);
        if(contourarea<GlobalConfig::getinstance().lightobj.min_area||
            contours[i].size()<GlobalConfig::getinstance().lightobj.min_countersz) 
            continue;//面积太小或点数太少舍弃
        RotatedRect lightrec=fitEllipse(contours[i]);
        adjustrec(lightrec);//规范化
        if(lightrec.angle>GlobalConfig::getinstance().lightobj.angle_range.first&&
            lightrec.angle<GlobalConfig::getinstance().lightobj.angle_range.second) 
            continue;//舍弃歪曲过大的
        if(lightrec.size.height/lightrec.size.width<GlobalConfig::getinstance().lightobj.hw_ratio.first||
            lightrec.size.height/lightrec.size.width>GlobalConfig::getinstance().lightobj.hw_ratio.second) 
            continue;//舍弃高宽比不合适的
        // drawContours(drawing,contours,i,cv::Scalar(255,0,0),2,LINE_8,hierarchy,0);
        //放大灯条以利于后续装甲板匹配
        lightrec.size.height*=1.3;
        lightrec.size.width*=1.3;
        rect.push_back(LightbarDetector(lightrec));
    }
    // imshow("contours",drawing);
    return rect;
}