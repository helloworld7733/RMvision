#pragma once
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

using namespace cv;
using namespace std;

class LightbarDetector
{
public:
    LightbarDetector(){}
    LightbarDetector(const cv::RotatedRect& light);
    vector<Mat> Imagetransform(const Mat& frame);//bgr转hsv
    Mat Imageprocess(const vector<Mat>& vchannel);//二值化，blur, dilate
    void adjustrec(RotatedRect& elps);//外接椭圆规范化
    vector<LightbarDetector> findcontour(const Mat& frame);//包含初步筛选
    RotatedRect lightrect;
    float getangle() const {return angle;}
    float getheight() const {return height;}
    float getwidth() const {return width;}
    Point2f getcenter() const {return center;}
private:
    float height,width,angle,area;
    cv::Point2f center;

};
