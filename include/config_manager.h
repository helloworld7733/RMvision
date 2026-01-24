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

struct lightbar_para
{
    std::pair<float,float> hw_ratio={1.5,15};
    std::pair<int,int> angle_range={60,120};
    float min_area=10;
    int min_countersz=10;
    int thresh_upper=240;
    int thresh_down=200;
};



class GlobalConfig
{
public:
    static GlobalConfig& getinstance()//单例模式，极大减少内存占用
    {
        static GlobalConfig instance;
        return instance;
    }

    lightbar_para lightobj;//组合关系
private:
    GlobalConfig(){}//构造函数私有化，使得只能通过单例模式访问
};