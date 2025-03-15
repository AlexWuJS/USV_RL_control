#include <iostream>
using namespace std;
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include "point_nav.h"
#include "control/Motor.h"
#include "tcp/tcp_connection.h"
#include "tcp/ship_info.h"
#include "tcp/keyboard.h"
#include "adjust_pid/pid.h"




void Point_Nav::run(sensor_msgs::NavSatFix self_point){
    AnglePID AnglePid;
    PID PositionPid;

    double now_angle = get_angle(self_point, point_list[_index]);
    double now_dist = get_distance(self_point, point_list[_index]);

    double angle_pid_output = AnglePid.update(desired_angle, now_angle);
    double position_pid_output = PositionPid.update(_desired_dist, now_angle);

    _left_thruster_pwm = THRUSTER_MID_PWM - angle_pid_output * 0.5 + location_pid_output * 0.5;
    _right_thruster_pwm = THRUSTER_MID_PWM + angle_pid_output * 0.5 + location_pid_output * 0.5;

    update_point(now_dist);
}

double Point_Nav::get_angle(sensor_msgs::NavSatFix self_point, sensor_msgs::NavSatFix waypoint){
    float now_lat_rad = now_lat * M_PI / 180;
    float desired_lat_rad = desired_lat * M_PI / 180;
    float delta_lat = (waypoint.latitude - self_point.latitude) * M_PI / 180;
    float delta_lon = (waypoint.longitude - self_point.longitude) * M_PI / 180;

    float y = sin(delta_lon) * cos(desired_lat_rad);
    float x = cos(now_lat_rad) * sin(desired_lat_rad) - sin(now_lat_rad) * cos(desired_lat_rad) * cos(delta_lon);
    float bear = atan2(y, x);
    bear = bear * 180 / M_PI;
    return bear;
}

double Point_Nav::get_distance(sensor_msgs::NavSatFix self_point, sensor_msgs::NavSatFix waypoint){
    const float R = 6371e3; // 地球半径，

    float now_lat_rad = now_lat * M_PI / 180;
    float desired_lat_rad = desired_lat * M_PI / 180;
    float delta_lat = (waypoint.latitude- self_point.latitude) * M_PI / 180;
    float delta_lon = (waypoint.longitude - self_point.longitude) * M_PI / 180;

    float a = sin(delta_lat / 2) * sin(delta_lat / 2) + cos(now_lat_rad) * cos(desired_lat_rad) * sin(delta_lon / 2) * sin(delta_lon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float dist = R * c;

    return dist;
}

void Point_Nav::update_point(double dist){
    // std::cout << "dist:" << dist << std::endl;
    if (dist  < _switch_dist) {
        std::cout << "go to next point" << std::endl;
        if (_index+1 == point_list.size()) { //this is the last waypoint
            std::cout << "this is the last waypoint" << std::endl;
            if (dist  < 1) {
                std::cout << "stop" << std::endl;
                is_final = true;
                
                //cout << "current speed:" << vel_b(0) << endl;
            }
        } else {
            _index++;
        }
    }
}

bool Point_Nav::recv_first_waypoint(sensor_msgs::NavSatFix waypoint){
        return (msg.latitude == 91 && msg.longtitude == 181);
}

bool Point_Nav::recv_last_waypoint(sensor_msgs::NavSatFix waypoint){
        return (msg.latitude == -91 && msg.longtitude == -181);
}

void Point_Nav::add_waypoint(sensor_msgs::NavSatFix waypoint){
    _temp_point_list.push_back(waypoint);
}

bool Point_Nav::begin_recv(sensor_msgs::NavSatFix waypoint){
    _temp_point_list.clear();
}

void Point_Nav::update_recv_waypoints(void){
    unordered_map<sensor_msgs::NavSatFix, int> hash_table;

    for (auto& waypoint : point_list){
        int waypoint_num = 0;
        hash_table.insert({waypoint, waypoint_num++});
    }
    for (auto& waypoint : _temp_point_list){
        int waypoint_num = 0;
        std::pair result = hash_table.insert({waypoint, waypoint_num++});
        if (result.second() && _index >= result.first().second()){// 回到第一个不同的点开始跟踪
                _index = result.first().second();
            break;
        }    
    }    
    point_list = std::move(_temp_point_list);  
}

std::pair<int, int> Point_Nav::get_thruster_pwm(void){
    return {_left_thruster_pwm, _right_thruster_pwm};
}