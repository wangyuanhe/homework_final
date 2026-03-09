#ifndef _WORKER_H_
#define _WORKER_H_

#include "tools.h"
#include "target_kf.h"
#include "serial_port.h"
#include "target_ope.h"

using namespace cv;
using namespace std;

class Homework : public rclcpp::Node{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscribe;
    rclcpp::Time last_fire_time;
    rclcpp::Time last_image_time;
    Point gun_pos;
    unique_ptr<Serialport> serial_port;

    KalmanTracker kft_tar;
    KalmanTracker kft_fri;
private:
    inline bool check_team_fuc(const Mat& work_image){
        static bool check_team = false;
        static bool res_team = false;
        if(!check_team){
            check_team = true;
            Vec3b bgr = work_image.at<Vec3b>(gun_pos.y,gun_pos.x);
            res_team = bgr[0] > bgr[2];
            RCLCPP_INFO(this->get_logger(),"MY TEAM IS %d",res_team);

        }
        return res_team;
    }
private:
    inline void find_targets_fuc(const Mat& work_image,const bool my_team,vector<Point>& my_targets,vector<Point>& my_friends){
        Mat gray_image;
        cvtColor(work_image,gray_image,CV_BGR2GRAY);
        threshold(gray_image,gray_image,100,255,CV_THRESH_BINARY_INV);

        vector<vector<Point>> contours;
        vector<Vec4i> hierarcy;
        findContours(gray_image,contours,hierarcy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

        my_friends.clear();
        my_targets.clear();

        for(int i = 0;i < contours.size();i++){
            Rect boundRect = boundingRect(Mat(contours[i]));
            
            Point top_left = boundRect.tl(); 
            Point bottom_right = boundRect.br();

            if(bottom_right.x - top_left.x < 64)continue;
            if(bottom_right.y > 612)continue;

            int cnt_b = 0,cnt_r = 0;
            bool gray_tag = false;
            for(int cl = top_left.x;cl <= min(top_left.x + 5,1151);cl++)
                for(int rw = top_left.y;rw <= bottom_right.y;rw++){
                    Vec3b bgr = work_image.at<Vec3b>(rw,cl);
                    cnt_b += bgr[0];cnt_r += bgr[2];
                    if(check_color(bgr,145,145,145,5))gray_tag = true;
                }
            for(int cl = max(bottom_right.x - 5,0);cl <= bottom_right.x;cl++)
                for(int rw = top_left.y;rw <= bottom_right.y;rw++){
                    Vec3b bgr = work_image.at<Vec3b>(rw,cl);
                    cnt_b += bgr[0];cnt_r += bgr[2];
                    if(check_color(bgr,145,145,145,5))gray_tag = true;
                }
            if(gray_tag)continue;

            if((cnt_b > cnt_r) != my_team){
                my_targets.push_back(Point((top_left.x + bottom_right.x) / 2,(top_left.y + bottom_right.y) / 2));
            }else{
                my_friends.push_back(Point((top_left.x + bottom_right.x) / 2,(top_left.y + bottom_right.y) / 2));
            }
        }
    }
public:
    Homework(string port_name):Node("worker_node"),serial_port(std::make_unique<Serialport>(port_name,115200)){
        image_subscribe = this->create_subscription<sensor_msgs::msg::Image>
        ("/image_raw",10,bind(&Homework::image_callback,this,placeholders::_1));

        last_fire_time = this->now();
        last_image_time = this->now();

        gun_pos = Point(576,612);
    }
private:
    inline void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
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
            dt_image = 0.016;
        }
        last_image_time = current_time;

        //判断敌我双方
        static bool my_team = check_team_fuc(read_image);

        //找靶位+判断靶
        vector<Point> my_targets;
        vector<Point> my_friends;
        find_targets_fuc(read_image,my_team,my_targets,my_friends);

        //离散化 + kf
        kft_fri.updata(my_friends,dt_image);
        auto res_fri = kft_fri.get_data();

        kft_tar.updata(my_targets,dt_image);
        auto res_tar = kft_tar.get_data();

        if(res_tar.empty())return;

        vector<pair<Point,Point>>friend_lines;
        friend_lines.clear();

        current_time = this->now();
        if(!res_fri.empty()){
            for(auto & target : res_fri)
                cal_pre_line(target,gun_pos,(current_time - last_image_time).seconds(),friend_lines);
        }

        double min_dist = 1e6;
        Point select_target = Point(0,0);
        for(auto& target : res_tar){
            Point cal_tar_tmp = cal_pre_tar(target,gun_pos,(current_time - last_image_time).seconds());
            
            bool line_tag = false;
            if(!res_fri.empty()){
                for(auto & line : friend_lines)
                    line_tag |= check_cross(gun_pos,cal_tar_tmp,line.first,line.second);
            }
            if(line_tag)continue;

            double tmp_dist = dis_point_Euc(gun_pos,cal_tar_tmp);
            if(tmp_dist < min_dist){
                min_dist = tmp_dist;
                select_target = cal_tar_tmp;
            }
        }
        if(select_target.x == 0 && select_target.y == 0)return;

        current_time = this->now();
        if((current_time - last_fire_time).seconds() > 0.1){
            //计算角度
            double angle = cal_angle(select_target,gun_pos);
            send_angle_com(angle);

            send_fire_com();
            send_fire_com();
            send_fire_com();
            last_fire_time = current_time;
        }
    }

    inline void send_angle_com(const double angle){
        uint8_t buffer[5];
        buffer[0] = 0x01;
        float angle_f = static_cast<float>(angle);
        memcpy(buffer + 1,&angle_f,4);
        serial_port->write(buffer,5);
    }
    
    inline void send_fire_com(){
        uint8_t buffer[1] = {0x02};
        serial_port->write(buffer, 1);
    }
};

#endif