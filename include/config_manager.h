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
    std::pair<float,float> hw_ratio={2,5};
    std::pair<int,int> angle_range={12,165};
    string enemy_color="red";//指定地方颜色为红色或蓝色
    float min_area=16;
    int min_countersz=15;
    int thresh_upper=240;
    int thresh_down=200;
};

struct armor_para
{
    float angle_diff=7;
    float heightratio_diff=0.1;
    float centeryratio_diff=0.1;
    float l_ratio=0.9;//允许的最小宽长比
    float b_ratio=1.7;//超过此范围的有效宽长比认定为大装甲，反之小装甲
    float t_ratio=2.2;//允许的最大宽长比
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
    armor_para armorobj;
private:
    GlobalConfig(){}//构造函数私有化，使得只能通过单例模式访问
};