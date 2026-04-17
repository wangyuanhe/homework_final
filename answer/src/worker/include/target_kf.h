#ifndef _TARGET_KF_H_
#define _TARGET_KF_H_

#include "tools.h"

using namespace std;
using namespace cv;

const int state_size = 6;
const int meas_size = 2;
const int cont_size = 0;

const float kQPos = 1e-5f;   // 位置噪声
const float kQVel = 1e-4f;   // 速度噪声
const float kQAcc = 1e-2f;   // 加速度噪声

const float kRPos = 1e-3f;   // 测量噪声

struct Target{
    int id,lost_count;
    KalmanFilter kf;
    Point pos,vel,acc;
    int health;
    double dt;

    Target(int _id,const Point& init_pos,double _dt = 1.0){
        id = _id;
        lost_count = 0;
        dt = _dt;
        health = 3;
        kf = KalmanFilter(state_size,meas_size,cont_size,CV_32F);
        
        // 状态转移矩阵 A
        cv::setIdentity(kf.transitionMatrix, cv::Scalar(1));
        kf.transitionMatrix.at<float>(0, 2) = dt;
        kf.transitionMatrix.at<float>(0, 4) = 0.5 * dt * dt;
        kf.transitionMatrix.at<float>(1, 3) = dt;
        kf.transitionMatrix.at<float>(1, 5) = 0.5 * dt * dt;
        kf.transitionMatrix.at<float>(2, 4) = dt;
        kf.transitionMatrix.at<float>(3, 5) = dt;

        // 测量矩阵 H
        kf.measurementMatrix = cv::Mat::zeros(meas_size, state_size, CV_32F);
        kf.measurementMatrix.at<float>(0, 0) = 1.0f;
        kf.measurementMatrix.at<float>(1, 1) = 1.0f;

        // 过程噪声协方差 Q
        cv::setIdentity(kf.processNoiseCov, cv::Scalar(0));
        kf.processNoiseCov.at<float>(0, 0) = kQPos;
        kf.processNoiseCov.at<float>(1, 1) = kQPos;
        kf.processNoiseCov.at<float>(2, 2) = kQVel;
        kf.processNoiseCov.at<float>(3, 3) = kQVel;
        kf.processNoiseCov.at<float>(4, 4) = kQAcc;
        kf.processNoiseCov.at<float>(5, 5) = kQAcc;

        // 测量噪声协方差 R
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(kRPos));

        // 初始状态
        cv::Mat state(state_size, 1, CV_32F);
        state.at<float>(0) = init_pos.x;
        state.at<float>(1) = init_pos.y;
        state.at<float>(2) = 0;
        state.at<float>(3) = 0;
        state.at<float>(4) = 0;
        state.at<float>(5) = 0;
        kf.statePost = state;

        // 初始协方差
        cv::setIdentity(kf.errorCovPost, cv::Scalar(100));
        kf.errorCovPost.at<float>(2,2) = 500;  // vx
        kf.errorCovPost.at<float>(3,3) = 500;  // vy
        kf.errorCovPost.at<float>(4,4) = 1000; // ax
        kf.errorCovPost.at<float>(5,5) = 1000; // ay

        pos = init_pos;
        vel = cv::Point(0,0);
        acc = cv::Point(0,0);
    }
    void predict(double _dt = 1.0){
        dt = _dt;
        kf.transitionMatrix.at<float>(0, 2) = dt;
        kf.transitionMatrix.at<float>(0, 4) = 0.5 * dt * dt;
        kf.transitionMatrix.at<float>(1, 3) = dt;
        kf.transitionMatrix.at<float>(1, 5) = 0.5 * dt * dt;
        kf.transitionMatrix.at<float>(2, 4) = dt;
        kf.transitionMatrix.at<float>(3, 5) = dt;

        Mat pred_pos = kf.predict();
        pos.x = static_cast<int>(pred_pos.at<float>(0));
        pos.y = static_cast<int>(pred_pos.at<float>(1));
        vel.x = static_cast<int>(pred_pos.at<float>(2));
        vel.y = static_cast<int>(pred_pos.at<float>(3));
        acc.x = static_cast<int>(pred_pos.at<float>(4));
        acc.y = static_cast<int>(pred_pos.at<float>(5));
    }
    void correct(const Point& meas_pos){
        Mat z(meas_size,1,CV_32F);
        z.at<float>(0) = meas_pos.x;
        z.at<float>(1) = meas_pos.y;
        Mat est = kf.correct(z);
        pos.x = static_cast<int>(est.at<float>(0));
        pos.y = static_cast<int>(est.at<float>(1));
        vel.x = static_cast<int>(est.at<float>(2));
        vel.y = static_cast<int>(est.at<float>(3));
        acc.x = static_cast<int>(est.at<float>(4));
        acc.y = static_cast<int>(est.at<float>(5));
    }
};
class KalmanTracker{
private:
    vector<Target>targets;
    int targets_cnt;
    int max_lost_count;
    double match_dis_threshold;
    double dt;

    void target_matching(const vector<Point>& poses,
                        vector<pair<int,int>>& matches,
                        vector<int>& unmatched_pos,
                        vector<int>& unmatched_tar){
        matches.clear();
        int n_pos = poses.size();
        int n_tar =targets.size();

        std::vector<bool> pos_matched(n_pos, false);
        std::vector<bool> tar_matched(n_tar, false);

        for(int i = 0;i < n_pos;i++){
            double min_dist = match_dis_threshold;
            int best_match = -1;
            for(int j = 0;j < n_tar;j++){
                if(tar_matched[j])continue;
                double tmp_dist = dis_point_Euc(poses[i],targets[j].pos);
                if(tmp_dist < min_dist){
                    min_dist = tmp_dist;
                    best_match = j;
                }
            }
            if(best_match != -1){
                matches.emplace_back(i,best_match);
                pos_matched[i] = true;
                tar_matched[best_match] = true;
            }
        }

        for(int i = 0;i < n_pos;i++)
            if(!pos_matched[i])unmatched_pos.push_back(i);
        for(int i = 0;i < n_tar;i++)
            if(!tar_matched[i])unmatched_tar.push_back(i);
    }
    inline void create_target(const Point& pos){
        targets.emplace_back(targets_cnt++,pos,dt);
    }
    inline void auto_remove(){
        targets.erase(remove_if(targets.begin(),targets.end(),
        [this](const Target& t){return (t.lost_count > max_lost_count || t.health < 0);}),targets.end());
    }
public:
    KalmanTracker(double _dt = 1.0){
        targets_cnt = 0;
        max_lost_count = 5;
        match_dis_threshold = 32.0;
        targets.clear();
        dt = _dt;
    }
    ~KalmanTracker(){
        targets_cnt = 0;
        max_lost_count = 0;
        match_dis_threshold = 0.0;
        dt = 0.0;
        targets.clear();
    }
    int point_matching(const Point point){
        double min_dist = match_dis_threshold;
        int n_tar =targets.size();
        int best_match = -1;
        for(int j = 0;j < n_tar;j++){
            double tmp_dist = dis_point_Euc(point,targets[j].pos);
            if(tmp_dist < min_dist){
                min_dist = tmp_dist;
                best_match = j;
            }
        }
        return best_match;
    }
    void health_ope(int id){
        targets[id].health--;
    }
    void updata(const vector<Point>& poses,double _dt = 1.0){
        dt = _dt;
        for(auto& target : targets)
            target.predict(dt);

        vector<pair<int,int>> matches;
        vector<int> unmatched_pos,unmatched_tar;

        target_matching(poses,matches,unmatched_pos,unmatched_tar);

        for(const auto& match : matches){
            int tmp1 = match.first;
            int tmp2 = match.second;
            targets[tmp2].correct(poses[tmp1]);
            targets[tmp2].lost_count = 0;
        }
        for(int i : unmatched_tar)
            targets[i].lost_count++;
        for(int i : unmatched_pos)
            create_target(poses[i]);

        auto_remove();
    }
    inline vector<Target> get_data()const{return targets;}
};

#endif