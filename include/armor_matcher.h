#pragma once
#include<iostream>
#include<windows.h>
#include<vector>
#include<iomanip>
#include<string>
#include<algorithm>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include<opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include<opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include<opencv2/calib3d.hpp>
#include"lightbar_detector.h"
using namespace cv;
using namespace std;


class Armor
{
public:
    Armor(){}
    //依据左右灯条设置角点坐标，顺序：左下，左上，右上，右下
    void setvertices(const LightbarDetector& leftbar, const LightbarDetector& rightbar);
    //计算对角线交点
    Point2f calc_cross(vector<Point2f> vertices);
    //将对角线交点作为中心点
    Point2f Crossline(const LightbarDetector& leftbar, const LightbarDetector& rightbar);
    //依据左右灯条构造装甲板
    Armor(const LightbarDetector& leftbar, const LightbarDetector& rightbar);
    
    int num;//数字识别结果
    float angle;//装甲板倾斜角
    cv::Point2f center;//装甲板中心
    vector<Point2f> vertices;//角点：左下、左上、右上、右下
    bool bflag;//true表示是大装甲，反之是小装甲
    int leftbar_index;//左灯条序号
    int rightbar_index;//右灯条序号
    float wh_ratio;//宽度差比率，用来筛选装甲板中的重复灯条
    
private:
    LightbarDetector leftlightbar;//一个装甲板由左右两个灯条构成，体现组合关系
    LightbarDetector rightlightbar;
};



class ArmorDetector
{
public:
    ArmorDetector(){}

    vector<Armor> matchbars(const vector<LightbarDetector>& lights);
    void Erase_repeats(vector<Armor>& armors);//去除含重复匹配灯条的装甲板
    void Erase_wrong(vector<Armor>& armors);
    void Frame_tracking(Mat frame, Mat old_frame, vector<Armor> old_armors, vector<Armor>& armors);//跨帧一致性约束
};