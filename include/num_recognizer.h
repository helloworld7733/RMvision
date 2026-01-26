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
#include <opencv2/ml.hpp>
#include<opencv2/calib3d.hpp>
#include"lightbar_detector.h"
#include"armor_matcher.h"
using namespace cv;
using namespace std;
using namespace cv::ml;


class Numrecognizer
{
public:
    Numrecognizer();
    void Loadsvm(string path);
    void Loadarmor(Armor& armor,const Mat& frame);
    void num_recognize();

private:
    Armor* armor_ptr;//组合
    Ptr<SVM> svm;
    cv::Size armorimgsz;//svm识别的图片大小
    Mat armorroi;//装甲板roi
    
};