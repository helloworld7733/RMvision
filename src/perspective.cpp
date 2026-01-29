#pragma once
#define PI 3.1415926
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
#include"perspective.h"

using namespace cv;
using namespace std;

//相机参数导入
Pnpsolver::Pnpsolver(string path)
{
    cv::FileStorage fs(path,FileStorage::READ);

    //异常处理
    if(!fs.isOpened())
    {
        cerr<<"fail in opening the camera para file"<<endl;
        exit(-1);
    }

    fs["camera_matrix"]>>mtx;
    fs["distortion_coefficients"]>>dist;

    mtx.convertTo(mtx,CV_64FC1);
    dist.convertTo(dist,CV_64FC1);

    //调整成正确的形状,mtx:3*3,dist:1*5
    // mtx=mtx.reshape(1,3);//1通道，3行
    // dist=dist.reshape(1,1);//1通道，1行
}

//得到旋转向量与平移向量
void Pnpsolver::Getvec(Armor& armor)
{
    float halfw,halfh;
    //赋予真实尺寸，使得在实际中能得到真实物理距离
    if(armor.bflag)//是大装甲
    {
        halfw=230/2;
        halfh=127/2;
    }
    else 
    {
        halfw=135/2;
        halfh=125/2;
    }

    vector<Point3f> obj_p;
    obj_p.push_back(Point3f(-halfw,halfh,0));
    obj_p.push_back(Point3f(-halfw,-halfh,0));
    obj_p.push_back(Point3f(halfw,-halfh,0));
    obj_p.push_back(Point3f(halfw,halfh,0));


    bool success=solvePnP(obj_p,armor.vertices,mtx,dist,rvec,tvec);
    if(!success)//异常处理
    {
        cerr<<"fail in pnp"<<endl;
        exit(-1);
    }
}

//平移向量可用于计算距离
void Pnpsolver::Practical_info()
{
    double tx=tvec.at<double>(0);
    double ty=tvec.at<double>(1);
    double tz=tvec.at<double>(2);

    distance=sqrt(tx*tx+ty*ty+tz*tz);//直线距离

    distance=distance/1000;//转为米单位
    
}