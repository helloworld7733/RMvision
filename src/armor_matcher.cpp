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

#include"lightbar_detector.h"
#include"config_manager.h"
#include"armor_matcher.h"

using namespace cv;
using namespace std;

void Armor::setvertices(const LightbarDetector& leftbar, const LightbarDetector& rightbar)
{
    cv::Size lsize((int)leftbar.lightrect.size.width,(int)leftbar.lightrect.size.height);
    cv::Size rsize((int)rightbar.lightrect.size.width,(int)rightbar.lightrect.size.height);
    RotatedRect lrect(leftbar.lightrect.center,lsize,leftbar.lightrect.angle);
    RotatedRect rrect(rightbar.lightrect.center,rsize,rightbar.lightrect.angle);
    cv::Point2f ptsl[4];
    lrect.points(ptsl);    
    cv::Point2f ptsr[4];
    rrect.points(ptsr);
    // vector<Point2f> vers;
    vertices.clear();
    vertices.push_back(ptsl[3]);
    vertices.push_back(ptsl[2]);
    vertices.push_back(ptsr[1]);
    vertices.push_back(ptsr[0]);
}

Point2f Armor::Crossline(const LightbarDetector& leftbar, const LightbarDetector& rightbar)
{
    setvertices(leftbar,rightbar);
    vector<Point2f> vers=vertices;
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

Armor::Armor(const LightbarDetector& leftbar, const LightbarDetector& rightbar)
{
    leftlightbar=leftbar;
    rightlightbar=rightbar;

    center=Crossline(leftbar,rightbar);//对角线为中心
    
    num=0;//暂时设0

    angle=(leftbar.getangle()+rightbar.getangle())/2;
}

void ArmorDetector::Erase_repeats(vector<Armor>& armors)
{
    for(int i=0;i<armors.size();i++)
    {
        for(int j=i+1;j<armors.size();j++)
        {
            if(armors[i].leftbar_index==armors[j].rightbar_index||
                armors[i].leftbar_index==armors[j].leftbar_index||
                armors[i].rightbar_index==armors[j].rightbar_index||
                armors[i].rightbar_index==armors[j].leftbar_index)
            {
                if(armors[i].hratio_diff<armors[j].hratio_diff)//去除y差比率大的armor
                {
                    armors.erase(armors.begin()+j);
                }
                else 
                {
                    armors.erase(armors.begin()+i);
                }
            }
        }
    }
}

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
            armor.hratio_diff=abs(lights[i].getcenter().y-lights[j].getcenter().y)/min(lights[i].getcenter().y,lights[j].getcenter().y);
            if(lights[i].getcenter().x<lights[j].getcenter().y)//分左右
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
    Erase_repeats(armors);
    return armors;
}