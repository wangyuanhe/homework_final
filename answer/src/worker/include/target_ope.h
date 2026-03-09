#ifndef _TARGET_OPE_H_
#define _TARGET_OPE_H_

#include "tools.h"

inline Point cal_pre_tar(const Target& target,const Point& gun_pos,double dt){
    Point pos = target.pos;
    Point vel = target.vel;
    Point acc = target.acc;

    pos.y += 16;
    Point p1 = cal_pre_point(pos,vel,acc,gun_pos,dt);

    return p1;
}
inline void cal_pre_line(const Target& target,const Point& gun_pos,double dt,vector<pair<Point,Point>>&friend_lines){
    Point pos = target.pos;
    Point vel = target.vel;
    Point acc = target.acc;

    pos.y += 16;
    pos.x -= 32;
    Point p1 = cal_pre_point(pos,vel,acc,gun_pos,dt);

    pos.x += 64;
    Point p2 = cal_pre_point(pos,vel,acc,gun_pos,dt);

    friend_lines.emplace_back(p1,p2);

    pos.y -= 32;
    pos.x -= 64;
    Point p3 = cal_pre_point(pos,vel,acc,gun_pos,dt);

    pos.x += 64;
    Point p4 = cal_pre_point(pos,vel,acc,gun_pos,dt);

    friend_lines.emplace_back(p3,p4);

}
#endif