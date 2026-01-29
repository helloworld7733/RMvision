#include<iostream>
#include<windows.h>
#include<vector>
#include<iomanip>
#include<algorithm>
#include<string>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include<opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include<opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include<opencv2/calib3d.hpp>

#include"lightbar_detector.h"
#include"config_manager.h"
#include"armor_matcher.h"

using namespace cv;
using namespace std;

//函数介绍：依据左右灯条设置出装甲板四个角点
void Armor::setvertices(const LightbarDetector& leftbar, const LightbarDetector& rightbar)
{
    cv::Size lsize((int)leftbar.lightrect.size.width,(int)leftbar.lightrect.size.height);
    cv::Size rsize((int)rightbar.lightrect.size.width,(int)rightbar.lightrect.size.height);
    //描出矩形框
    RotatedRect lrect(leftbar.lightrect.center,lsize,leftbar.lightrect.angle);
    RotatedRect rrect(rightbar.lightrect.center,rsize,rightbar.lightrect.angle);
    cv::Point2f ptsl[4];
    lrect.points(ptsl);    
    cv::Point2f ptsr[4];
    rrect.points(ptsr);

    vertices.clear();
    //大坑：由于rotatedrect.points返回点顺序不固定，故此处依据实际情况进行校对
    //最终保证顺序：左下，左上，右上，右下
    if(ptsl[1].x<ptsl[2].x)
    {
        vertices.push_back(ptsl[3]);
        vertices.push_back(ptsl[2]);
    }
    else 
    {
        vertices.push_back(ptsl[1]);
        vertices.push_back(ptsl[0]);
    }
    if(ptsr[1].x<ptsr[2].x)
    {
        vertices.push_back(ptsr[1]);
        vertices.push_back(ptsr[0]);
    }
    else 
    {
        vertices.push_back(ptsr[3]);
        vertices.push_back(ptsr[2]);
    }
}

//函数介绍：计算四个点对角线坐标
Point2f Armor::calc_cross(vector<Point2f> vertices)
{
    vector<Point2f> vers=vertices;
    //依据四个点计算对角线坐标，使用y=kx+b形式计算，简单数学原理
    float k1=(vers[2].y-vers[0].y)/(vers[2].x-vers[0].x);
    float k2=(vers[1].y-vers[3].y)/(vers[1].x-vers[3].x);
    float cross1=vers[0].y*vers[2].x-vers[2].y*vers[0].x;
    float b1=vers[0].y-k1*vers[0].x;
    float cross2=vers[1].y*vers[3].x-vers[3].y*vers[1].x;
    float b2=vers[1].y-k2*vers[1].x;
    float retx=(b2-b1)/(k1-k2);
    float rety=k1*retx+b1;
    Point2f ret(retx,rety);
    return ret;
}

//函数介绍：将角点对角线设置为装甲板中心
Point2f Armor::Crossline(const LightbarDetector& leftbar, const LightbarDetector& rightbar)
{
    setvertices(leftbar,rightbar);
    //将四个点交点作为装甲板中心点
    return calc_cross(vertices);
}

//函数介绍：装甲板构造函数
Armor::Armor(const LightbarDetector& leftbar, const LightbarDetector& rightbar)
{
    leftlightbar=leftbar;
    rightlightbar=rightbar;

    center=Crossline(leftbar,rightbar);//对角线为中心
    
    //数字识别结果，暂时设0
    num=0;

    //角度采用左右灯条角度平均值
    angle=(leftbar.getangle()+rightbar.getangle())/2;
}

//函数介绍：去除在同时匹配上同一个灯条的不同装甲板之一，因为一个灯条只能有一个装甲板匹配
void ArmorDetector::Erase_repeats(vector<Armor>& armors)
{
    vector<int> repeats;
    float sl=GlobalConfig::getinstance().armorobj.l_ratio;
    float su=GlobalConfig::getinstance().armorobj.b_ratio;
    float bu=GlobalConfig::getinstance().armorobj.t_ratio;
    for(int i=0;i<armors.size();i++)
    {
        for(int j=i+1;j<armors.size();j++)
        {
            //若不同装甲板你左右灯条下标一样，意味着匹配到相同灯条
            if(armors[i].leftbar_index==armors[j].rightbar_index||
                armors[i].leftbar_index==armors[j].leftbar_index||
                armors[i].rightbar_index==armors[j].rightbar_index||
                armors[i].rightbar_index==armors[j].leftbar_index)
            {
                //筛选标准：在两个装甲板宽高比差在合理范围内时，去除偏角大的；不在合理范围时，去除宽长比大的
                if(abs(armors[i].wh_ratio-armors[j].wh_ratio)<GlobalConfig::getinstance().armorobj.min_whratio)
                {
                    cout<<777<<endl;
                    if(armors[i].angle<armors[j].angle)
                        repeats.push_back(j);
                    else 
                        repeats.push_back(i);
                }
                else 
                {
                    if(armors[i].wh_ratio>sl&&armors[i].wh_ratio<bu)//去除height差比率大的armor
                    {
                        if(armors[i].wh_ratio<armors[j].wh_ratio)
                            repeats.push_back(j);
                        else 
                            repeats.push_back(i);
                    }
                    else     
                        repeats.push_back(i);
                }
            }
        }
    }
    //先把需要去除的装甲板你序号放入repeats中，此处使用vector.erase从后往前去除（防止从前往后时，后面元素往前进而导致下标不匹配问题）
    sort(repeats.begin(),repeats.end());
    repeats.erase(unique(repeats.begin(),repeats.end()),repeats.end());
    for(int i=repeats.size()-1;i>=0;i--)
    {
        armors.erase(armors.begin()+repeats[i]);//注意往前移了
    }
}

//函数介绍：去除错误匹配，用于普通情况下与Klt光流检测后筛去显然不合理的装甲板
void ArmorDetector::Erase_wrong(vector<Armor>& armors)
{
    vector<int> repeats;
    for(int i=0;i<armors.size();i++)
    {
        float dis01=cv::norm(armors[i].vertices[0]-armors[i].vertices[1]);
        float dis02=cv::norm(armors[i].vertices[1]-armors[i].vertices[2]);
        float dis03=cv::norm(armors[i].vertices[2]-armors[i].vertices[3]);
        float dis04=cv::norm(armors[i].vertices[3]-armors[i].vertices[0]);
        float maxdis=max({dis01,dis02,dis03,dis04});
        float mindis=min({dis01,dis02,dis03,dis04});
        if(armors[i].bflag)//大装甲
        {
            //筛选标准1：若装甲板四个边中最大边比最小边大的多，则认定不合理
            if((maxdis-mindis)/maxdis>GlobalConfig::getinstance().armorobj.dis_ratio_b)
            {
                repeats.push_back(i);
            }
        }
        else 
        {
            if((maxdis-mindis)/maxdis>GlobalConfig::getinstance().armorobj.dis_ratio_s)
            {
                repeats.push_back(i);
            }
        }
        //筛选标准2：若装甲板左右两角点y相差很大，则装甲板可能出现扭曲，认定不合理
        if(abs(armors[i].vertices[1].y-armors[i].vertices[2].y)>GlobalConfig::getinstance().armorobj.y_diff)
            repeats.push_back(i);

    }
    //同erase_repeats一样的筛去逻辑
    sort(repeats.begin(),repeats.end());
    repeats.erase(unique(repeats.begin(),repeats.end()),repeats.end());
    for(int i=repeats.size()-1;i>=0;i--)
    {
        armors.erase(armors.begin()+repeats[i]);//注意往前移了
    }
}

//函数介绍：依据灯条匹配合适的装甲板
vector<Armor> ArmorDetector::matchbars(const vector<LightbarDetector>& lights)
{
    vector<Armor> armors;
    for(int i=0;i<lights.size();i++)
    {
        for(int j=i+1;j<lights.size();j++)
        {
            //灯条是否匹配判断方法：近似平行，长度相似，中心点y近似，宽长比
            //平行筛选
            if(abs(lights[i].getangle()-lights[j].getangle())>GlobalConfig::getinstance().armorobj.angle_diff)
                continue;
            //长度差比率筛选
            if(abs(lights[i].getheight()-lights[j].getheight())/min(lights[i].getheight(),lights[j].getheight())>
                    GlobalConfig::getinstance().armorobj.heightratio_diff)
                continue;
            //中心点y差比率筛选
            if(abs(lights[i].getcenter().y-lights[j].getcenter().y)/min(lights[i].getcenter().y,lights[j].getcenter().y)>
                    GlobalConfig::getinstance().armorobj.centeryratio_diff)
                continue;
            //宽长比筛选
            float h=(lights[i].getheight()+lights[j].getheight())/2;
            float w=sqrt(pow(lights[i].getcenter().x-lights[j].getcenter().x,2)+
                        pow(lights[i].getcenter().y-lights[j].getcenter().y,2));
            float ratio=w/h;
            if(ratio<GlobalConfig::getinstance().armorobj.l_ratio||
                ratio>GlobalConfig::getinstance().armorobj.t_ratio)
                continue;
            Armor armor;
            armor.wh_ratio=ratio;
            if(lights[i].getcenter().x<lights[j].getcenter().x)//分左右
            {    
                armor=Armor(lights[i],lights[j]);
                armor.leftbar_index=i;
                armor.rightbar_index=j;
            }
            else 
            {   
                armor=Armor(lights[j],lights[i]);
                armor.leftbar_index=j;
                armor.rightbar_index=i;
            }
            //大小装甲区别：小于b_ratio则是小装甲，反之大装甲
            if(ratio<GlobalConfig::getinstance().armorobj.b_ratio)
            {
                //认定为小装甲
                armor.bflag=false;
            }
            else 
            {
                //认定为大装甲
                armor.bflag=true;
            }
            armors.push_back(armor);
        }
    }
    //重复匹配筛选（不同装甲板匹配到相同灯条）与错误匹配二次筛选
    Erase_repeats(armors);
    Erase_wrong(armors);
    return armors;
}

//跨帧一致性约束：由于装甲板匹配可能在不同帧之间短暂性失效，此处使用Klt光流追踪法，对前一帧的装甲板四个角点进行追踪,能极大提高识别度
void ArmorDetector::Frame_tracking(Mat frame, Mat old_frame, vector<Armor> old_armors, vector<Armor>& armors)
{
    vector<Point2f> newvertices;
    vector<uchar> status;
    vector<float> err;
    bool tracking_success=true;
    for(int i=0;i<old_armors.size();i++)
    {
        //klt光流法追踪，追踪得到的新角点放入newvertices，每个点追踪状态放入status，误差放入err
        cv::calcOpticalFlowPyrLK(old_frame,frame,old_armors[i].vertices,newvertices,status,err);

        for(auto a:status)
        {
            if(!a) tracking_success=false;
        }
        //每个点都追踪成功，总的才能算成功
        if(tracking_success)
        {
            Armor armor;
            //提高准确率关键一招：追踪出的装甲板数字识别结果直接采用旧装甲板数字识别结果
            armor.num=old_armors[i].num;
            armor.vertices=newvertices;
            //大小装甲板参数不变
            armor.bflag=old_armors[i].bflag;
            armor.center=armor.calc_cross(newvertices);
            armors.push_back(armor);
        }
    }
    //由于klt光流法可能得到很离谱的匹配结果（实验中发现），故最后进行错误匹配筛选
    Erase_wrong(armors);
    
}