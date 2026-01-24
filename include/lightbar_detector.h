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
    vector<Mat> Imagetransform(const Mat& frame);//bgr转csv
    Mat Imageprocess(const vector<Mat>& vchannel);//二值化，blur, dilate
    void findcontour(const Mat& frame);//包含初步筛选

private:
    float height,width,angle,area;
    cv::Point2f center;

};