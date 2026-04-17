#ifndef _WORKER_H_
#define _WORKER_H_

#include "tools.h"
#include "target_kf.h"
#include "serial_port.h"
#include "target_ope.h"

using namespace cv;
using namespace std;

class Worker : public rclcpp::Node{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscribe;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr point_publisher;
    rclcpp::Time last_fire_time;
    rclcpp::Time last_image_time;
    Point gun_pos;

    KalmanTracker kft_tar;
    KalmanTracker kft_fri;

    bool check_team = false;

    bool IS_DEBUG = false;
    vector<double> health_impact_factor = {1e7,1.0,0.7,0.1};
    unique_ptr<Serialport> serial_port;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
public:
    Worker():Node("worker_node"){
        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) {
                for (const auto &p : params) {
                    if (p.get_name() == "health_impact_factor" && 
                        p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                        health_impact_factor = p.as_double_array();
                        RCLCPP_INFO(this->get_logger(), 
                            "health_impact_factor changed = %f,%f,%f,%f",
                            health_impact_factor[0], health_impact_factor[1],
                            health_impact_factor[2], health_impact_factor[3]);
                    }
                    if (p.get_name() == "debug" && 
                        p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                        IS_DEBUG = p.as_bool();
                        RCLCPP_INFO(this->get_logger(), "debug changed = %d", IS_DEBUG);
                    }
                    if (p.get_name() == "serial" && 
                        p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                        std::string port_name = p.as_string();
                        serial_port = std::make_unique<Serialport>(port_name, 115200);
                        check_team = false;
                        RCLCPP_INFO(this->get_logger(), "\033[2J\033[1;1H");
                        RCLCPP_INFO(this->get_logger(), "serial changed = %s", port_name.c_str());
                    }
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            }
        );

        this->declare_parameter<std::string>("serial","/dev/pts/2");
        this->declare_parameter<bool>("debug",false);
        this->declare_parameter<vector<double>>("health_impact_factor",{1e7,1.0,0.7,0.1});

        std::string port_name = this->get_parameter("serial").as_string();
        IS_DEBUG = this->get_parameter("debug").as_bool();
        health_impact_factor = this->get_parameter("health_impact_factor").as_double_array();

        serial_port = std::make_unique<Serialport>(port_name,115200);

        image_subscribe = this->create_subscription<sensor_msgs::msg::Image>
        ("/image_raw",10,bind(&Worker::image_callback,this,placeholders::_1));
        point_publisher = this->create_publisher<geometry_msgs::msg::Point32>
        ("/target_point",10);

        last_fire_time = this->now();
        last_image_time = this->now();

        gun_pos = Point(576,612);
    }
private:
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

        if(!res_tar.empty()){
            vector<pair<Point,Point>>friend_lines;
            friend_lines.clear();

            current_time = this->now();
            for(auto & target : res_fri)
                cal_pre_line(target,gun_pos,(current_time - last_image_time).seconds(),friend_lines);

            double min_dist = 1e9 + 10;
            Point select_target = Point(0,0);
            for(auto& target : res_tar){
                Point cal_tar_tmp = cal_pre_tar(target,gun_pos,(current_time - last_image_time).seconds());
                if(!check_scope(cal_tar_tmp,1152,648))continue;

                bool line_tag = false;
                for(auto & line : friend_lines)
                    line_tag |= check_cross(gun_pos,cal_tar_tmp,line.first,line.second);
                if(line_tag)continue;

                double tmp_dist = dis_point_Euc(gun_pos,cal_tar_tmp) * health_impact_factor[target.health];
                if(tmp_dist < min_dist){
                    min_dist = tmp_dist;
                    select_target = cal_tar_tmp;
                }
            }
            if(min_dist < 1e9){
                current_time = this->now();
                if((current_time - last_fire_time).seconds() > 0.16){
                    //计算角度
                    double angle = cal_angle(select_target,gun_pos);
                    serial_port->send_angle_com(angle);
                    serial_port->send_fire_com();

                    last_fire_time = current_time;
                
                    geometry_msgs::msg::Point32 pub;
                    pub.x = select_target.x;
                    pub.y = select_target.y;
                    point_publisher->publish(pub);
                }
            }

            if(IS_DEBUG && min_dist < 1e9){
                circle(read_image,select_target,3,Scalar(255,0,255),-1);
                cv::line(read_image,gun_pos,expand_line(gun_pos,select_target),Scalar(255,0,255));
                for(auto &line : friend_lines)
                    cv::line(read_image,line.first,line.second,Scalar(255,0,255));
            }
        }

        if(IS_DEBUG){
            for(auto &target : res_fri){
                circle(read_image,target.pos,3,Scalar(255,0,255),-1);
            }
            for(auto &target : res_tar){
                circle(read_image,target.pos,3,Scalar(0,255,0),-1);
                char s[1];s[0] = '0' + target.health;
                putText(read_image,s,target.pos,FONT_HERSHEY_COMPLEX,
                        1,Scalar(255,255,255));
            }
            imshow("DisPlay image",read_image);
            waitKey(15);
        }
    }
    bool check_team_fuc(const Mat& work_image){
        static bool res_team = false;
        if(!check_team){
            check_team = true;
            Vec3b bgr = work_image.at<Vec3b>(gun_pos.y,gun_pos.x);
            res_team = bgr[0] > bgr[2];
            RCLCPP_INFO(this->get_logger(),"MY TEAM IS %d",res_team);

        }
        return res_team;
    }
    void find_targets_fuc(const Mat& work_image,const bool my_team,vector<Point>& my_targets,vector<Point>& my_friends){
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
            if(gray_tag){
                int tmp = kft_tar.point_matching(Point((top_left.x + bottom_right.x) / 2,(top_left.y + bottom_right.y) / 2));
                if(tmp != -1){
                    my_targets.push_back(Point((top_left.x + bottom_right.x) / 2,(top_left.y + bottom_right.y) / 2));
                    kft_tar.health_ope(tmp);
                }else my_friends.push_back(Point((top_left.x + bottom_right.x) / 2,(top_left.y + bottom_right.y) / 2));
            
            }else if((cnt_b > cnt_r) != my_team){
                my_targets.push_back(Point((top_left.x + bottom_right.x) / 2,(top_left.y + bottom_right.y) / 2));
            }else{
                my_friends.push_back(Point((top_left.x + bottom_right.x) / 2,(top_left.y + bottom_right.y) / 2));
            }
        }
    }
};

#endif