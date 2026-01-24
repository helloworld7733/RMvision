#pragma once
#include<iostream>
#include<windows.h>
#include<vector>
#include<iomanip>
#include<string>

class LightbarDetector
{
public:
    LightbarDetector(float h,float w,float mina=15,std::pair<int,int> mangle={60,120},std::pair<float,float> hw_r={1.5,15},int mcountersz=10,int )
    {
        height=h;
        width=w;
    }

private:
    float min_area;//允许的最小面积、
    std::pair<float,float> hw_ratio;//允许的高宽比范围
    std::pair<int,int> angle;//允许的旋转角度
    int mincountersz;//允许的最小轮廓特征点数量
    float height,width,angle,area;
    cv::Point2f center;

};