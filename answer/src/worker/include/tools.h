#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <thread>
#include <iostream>
#include <memory>
#include <bits/stdc++.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_event_handler.hpp>

using namespace std;
using namespace cv;

inline bool point_cmp(Point a,Point b){
    return a.y < b.y;
}
inline bool check_color(const Vec3b& bgr,int B,int G,int R,int delta = 15){
    return (abs(bgr[0] - B) <= delta && abs(bgr[1] - G) <= delta && abs(bgr[2] - R) <= delta);
}
inline int dis_point_Manh(const Point& a,const Point& b){
    return abs(a.x - b.x) + abs(a.y - b.y);
}
inline double dis_point_Euc(const Point& a,const Point& b){
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}
inline bool check_scope(const Point& p,int rows,int cols){
    if(p.x < 0 || p.x > rows)return 0;
    if(p.y < 0 || p.y > cols)return 0;
    return 1;
}
inline double cal_angle(const Point& pos,const Point& pos2){
    double dx = pos.x - pos2.x;
    double dy = pos.y - pos2.y;
    double rad = atan2(dy,dx);
    double deg = rad * (-180.0) / CV_PI;
    return deg;
}
inline Point cal_pre_point(const Point& _pos,const Point& _vel,const Point& _acc,const Point& gun_pos,double dt){
    Point pos = _pos;
    Point vel = _vel;
    Point acc = _acc;

    double dist = dis_point_Euc(pos,gun_pos) + dt * 600.0;
    double t0 = dist / 600.0;

    for(int i = 0;i < 10;i++){
        Point pre = pos + vel * t0 + 0.5 * acc * t0 * t0;
        dist = dis_point_Euc(pre,gun_pos) + dt * 600.0;
        t0 = dist / 600.0;
    }
    return pos + vel * t0 + 0.5 * acc * t0 * t0;
}
inline int cross_product(const Point a,const Point b){
    return a.x * b.y - a.y * b.x;
}
inline bool check_cross(const Point l1_1,const Point l1_2,const Point l2_1,const Point l2_2){
    // 快速排斥实验
    if (max(l1_1.x, l1_2.x) < min(l2_1.x, l2_2.x) ||
        max(l1_1.y, l1_2.y) < min(l2_1.y, l2_2.y) ||
        max(l2_1.x, l2_2.x) < min(l1_1.x, l1_2.x) ||
        max(l2_1.y, l2_2.y) < min(l1_1.y, l1_2.y)) {
        return false;
    }

    // 跨立实验
    Point v1 = Point(l1_2.x - l1_1.x, l1_2.y - l1_1.y);  // 线段 l1 的方向向量
    Point v2 = Point(l2_2.x - l2_1.x, l2_2.y - l2_1.y);  // 线段 l2 的方向向量

    int cp1 = cross_product(Point(l1_1.x - l2_1.x, l1_1.y - l2_1.y), v2);  // (l1_1 - l2_1) × v2
    int cp2 = cross_product(Point(l1_2.x - l2_1.x, l1_2.y - l2_1.y), v2);  // (l1_2 - l2_1) × v2
    int cp3 = cross_product(Point(l2_1.x - l1_1.x, l2_1.y - l1_1.y), v1);  // (l2_1 - l1_1) × v1
    int cp4 = cross_product(Point(l2_2.x - l1_1.x, l2_2.y - l1_1.y), v1);  // (l2_2 - l1_1) × v1

    return (cp1 * cp2 <= 0 && cp3 * cp4 <= 0);
} 
inline Point expand_line(const Point p1,const Point p2){
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    Point res(p1.x - ((dx * p1.y) / dy),0);
    return res;
}
#endif