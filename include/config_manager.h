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

struct lightbar_para//灯条可调参数
{
    std::pair<float,float> hw_ratio={2,5};//合适的高宽比范围
    std::pair<int,int> angle_range={12,165};//合适的角度范围
    string enemy_color="red";//指定地方颜色为红色或蓝色
    float min_area=10;//允许的最小面积
    int min_countersz=10;//允许的最小点数
    int thresh_upper=240;//亮度阈值（未用上）
    int thresh_down=200;
};

struct armor_para
{
    //构造装甲板所筛选参数
    float angle_diff=8;//最大角度差
    float heightratio_diff=0.12;//高度比差
    float centeryratio_diff=0.12;//中心点y差
    float l_ratio=0.85;//允许的最小宽长比
    float b_ratio=1.85;//超过此范围的有效宽长比认定为大装甲，反之小装甲
    float t_ratio=2.3;//允许的最大宽长比

    //去除误测的装甲板
    float min_whratio=0.5;//最大宽高比差
    float min_angle=10;//最大角度差
    float dis_ratio_b=0.7;//最大边长比例差（大装甲）
    float dis_ratio_s=0.25;//最大边长比例差（小装甲）
    float y_diff=10;//最大y差
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