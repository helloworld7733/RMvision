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

//pnp位姿估计，获取实际小车距离信息
class Pnpsolver
{
public:
    //加载相机参数
    Pnpsolver(string path);
    //得到旋转向量与平移向量
    void Getvec(Armor& armor);
    //将向量信息转为实用信息
    void Practical_info();

    double distance;//直线距离

private:
    Mat dist;//畸变系数
    Mat mtx;//相机矩阵
    Mat tvec,rvec;//旋转向量和平移向量

};