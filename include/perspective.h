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
#include"num_recognizer.h"
#include"config_manager.h"

using namespace cv;
using namespace std;

class Pnpsolver
{
public:
    Pnpsolver(string path);

    void Getvec(Armor& armor);

    void Practical_info();

    double distance;//直线距离

private:
    Mat dist;
    Mat mtx;
    Mat tvec,rvec;//旋转向量和平移向量

};