#ifndef _DEBUGER_H_
#define _DEBUGER_H_

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

using namespace std;
using namespace cv;

inline double dis_point_Euc(const Point& a,const Point& b){
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

class Debuger : public rclcpp::Node{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscribe;
    rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr point_subscribe;
    
    rclcpp::Time last_image_time;

    vector<pair<Point,double>> mem_point;
    Point gun_pos;
public:
    Debuger():Node("debuger_node"){
        image_subscribe = this->create_subscription<sensor_msgs::msg::Image>
        ("/image_raw",10,bind(&Debuger::image_callback,this,placeholders::_1));
        point_subscribe = this->create_subscription<geometry_msgs::msg::Point32>
        ("/target_point",10,bind(&Debuger::point_callback,this,placeholders::_1));

        gun_pos = Point(576,612);

        last_image_time = this->now();
    }
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg,sensor_msgs::image_encodings::BGR8);
        Mat read_image = cv_ptr->image;
        if(read_image.empty()){
            RCLCPP_INFO(this->get_logger(), "ERROR(image empty)\n");
            return;
        }
        //帧间隔
        rclcpp::Time current_time = this->now();
        double dt_image = (current_time - last_image_time).seconds();
        if(dt_image < 0.0 || dt_image > 0.1){
            dt_image = 0.000;
        }
        last_image_time = current_time;
        RCLCPP_INFO(this->get_logger(), "============");
        RCLCPP_INFO(this->get_logger(), "dt_image = %lf",dt_image);

        for(auto &i : mem_point){
            circle(read_image,i.first,3,Scalar(0,255,0),-1);
            i.second -= dt_image;
            RCLCPP_INFO(this->get_logger(), "%lf",i.second);
        }
        mem_point.erase(remove_if(mem_point.begin(),mem_point.end(),
        [this](const pair<Point,double>a){
            return a.second < 0;
        }),mem_point.end());

        imshow("Debug Play image",read_image);
        waitKey(15);
    }
    void point_callback(const geometry_msgs::msg::Point32::SharedPtr msg){
        Point tmp;
        tmp.x = (*msg).x;
        tmp.y = (*msg).y;
        double wait_time = dis_point_Euc(tmp,gun_pos) / 600.0;

        mem_point.push_back(make_pair(tmp,wait_time));
    }
};

#endif