#include <iostream>
using namespace std;
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include "adjust_pid/point_nav.h"
#include "thruster_manager/Thruster.h"
#include "tcp/ship_info.h"
#include "tcp/joystick.h"


// 定义并初始化常量
const sensor_msgs::NavSatFix Point_Nav::virtual_start_waypoint = [] {
    sensor_msgs::NavSatFix pos;
    pos.latitude = 91.0;    // 默认纬度
    pos.longitude = 181.0;   // 默认经度
    pos.altitude = 0.0;    // 默认高度
    pos.header.frame_id = "map";  // 可选：设置坐标系
    return pos;
}();

const sensor_msgs::NavSatFix Point_Nav::virtual_end_waypoint = [] {
    sensor_msgs::NavSatFix pos;
    pos.latitude = -91.0;    // 默认纬度
    pos.longitude = -181.0;   // 默认经度
    pos.altitude = 0.0;    // 默认高度
    pos.header.frame_id = "map";  // 可选：设置坐标系
    return pos;
}();

void Point_Nav::get_nh_param(const ros::NodeHandle& nh){
    //pid
    if (!nh.getParam("/point_nav/angle_kp", AnglePid.kp))
    ROS_WARN("angle_kp not found");
    if (!nh.getParam("/point_nav/angle_ki", AnglePid.ki))
        ROS_WARN("angle_ki not found");
    if (!nh.getParam("/point_nav/angle_kd", AnglePid.kd))
        ROS_WARN("angle_kd not found");
    if (!nh.getParam("/point_nav/speed_kp", PositionPid.kp))
        ROS_WARN("speed_kp not found");
    if (!nh.getParam("/point_nav/speed_ki", PositionPid.ki))
        ROS_WARN("speed_ki not found");
    if (!nh.getParam("/point_nav/speed_kd", PositionPid.kd))
        ROS_WARN("speed_kd not found");
    nh.param<double>("/point_nav/desired_speed", _desired_speed, 0.5);
    if (!nh.getParam("/point_nav/maxIntegral", AnglePid.maxIntegral))
        ROS_WARN("angle maxIntegral not found");
    if (!nh.getParam("/point_nav/maxOutput", AnglePid.maxOutput))
        ROS_WARN("angle maxOutput not found");
    if (!nh.getParam("/point_nav/maxIntegral", PositionPid.maxIntegral))
        ROS_WARN("position maxIntegral not found");
    if (!nh.getParam("/point_nav/maxOutput", PositionPid.maxOutput))
        ROS_WARN("position maxOutput not found");
}

void Point_Nav::run(sensor_msgs::NavSatFix current_position, double now_angle, double current_speed){
    if (point_list.empty()){
        stop();
        ROS_DEBUG("point_list is empty");
        return;
    }
    
    double desired_angle = get_angle(current_position, point_list[_index]);
    double now_dist = get_distance(current_position, point_list[_index]);
    ROS_INFO("desired_angle: %f, now_dist: %f", desired_angle, now_dist);

    double angle_pid_output = angle_pid.update(desired_angle, now_angle);
    double speed_pid_output = speed_pid.update(_desired_speed, current_speed);

    _left_thruster_pwm = THRUSTER_MID_PWM - angle_pid_output * 0.5 + speed_pid_output * 0.5;
    _right_thruster_pwm = THRUSTER_MID_PWM + angle_pid_output * 0.5 + speed_pid_output * 0.5;

    update_point(now_dist);
}

double Point_Nav::get_angle(sensor_msgs::NavSatFix current_position, sensor_msgs::NavSatFix waypoint){
    float now_lat_rad = current_position.latitude * M_PI / 180;
    float desired_lat_rad = waypoint.latitude * M_PI / 180;
    float delta_lon_rad = (waypoint.longitude - current_position.longitude) * M_PI / 180;

    // east_component / north_component 是“从当前点指向航点”的方向在局部切平面上的东/北投影分量。
    // 注意：这不是经纬度直接转 ENU 坐标点位，只是计算“朝向”的方向分量。
    float east_component = sin(delta_lon_rad) * cos(desired_lat_rad); // 东向分量（East）
    float north_component =
        cos(now_lat_rad) * sin(desired_lat_rad) - sin(now_lat_rad) * cos(desired_lat_rad) * cos(delta_lon_rad); // 北向分量（North）

    // ENU yaw（REP-103）：0° 指向正东（+X），+90° 指向正北（+Y）
    float yaw_enu_deg = atan2(north_component, east_component) * 180 / M_PI; // 关键：atan2(N, E)

    // 归一化到 (-180, 180]，避免后续角度环出现 270° 这种跳变误差
    while (yaw_enu_deg > 180.0f) yaw_enu_deg -= 360.0f;
    while (yaw_enu_deg <= -180.0f) yaw_enu_deg += 360.0f;

    return yaw_enu_deg;
}

double Point_Nav::get_distance(sensor_msgs::NavSatFix current_position, sensor_msgs::NavSatFix waypoint){
    const float R = 6371e3; // 地球半径，单位：米

    float now_lat_rad = current_position.latitude * M_PI / 180;
    float desired_lat_rad = waypoint.latitude * M_PI / 180;
    float delta_lat = (waypoint.latitude - current_position.latitude) * M_PI / 180;
    float delta_lon = (waypoint.longitude - current_position.longitude) * M_PI / 180;

    float a = sin(delta_lat / 2) * sin(delta_lat / 2) + cos(now_lat_rad) * cos(desired_lat_rad) * sin(delta_lon / 2) * sin(delta_lon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float dist = R * c;

    return dist;
}

void Point_Nav::update_point(const double dist){
    // std::cout << "dist:" << dist << std::endl;
    if (dist  < _switch_dist) {
        if (_index == point_list.size()-1) { //this is the last waypoint
            if (dist  < 1) {
                std::cout << "stop" << std::endl;
                stop();
                // is_final = true;
            }else
                std::cout << "this is the last waypoint" << std::endl;
        } else {
            _index++;
            std::cout << "go to next point" << std::endl;
        }
    }
}

bool Point_Nav::recv_first_waypoint(sensor_msgs::NavSatFix waypoint) const {
    return (waypoint.latitude == virtual_start_waypoint.latitude && 
            waypoint.longitude == virtual_start_waypoint.longitude);
}

bool Point_Nav::recv_last_waypoint(sensor_msgs::NavSatFix waypoint) const {
    return (waypoint.latitude == virtual_end_waypoint.latitude && 
            waypoint.longitude == virtual_end_waypoint.longitude);
}

void Point_Nav::add_waypoint(sensor_msgs::NavSatFix waypoint){
    _temp_point_list.push_back(waypoint);
}

void Point_Nav::begin_recv(void){
    _temp_point_list.clear();
}

void Point_Nav::update_recv_waypoints(void){
    std::set<std::pair<float, float>> hash_table;

    if (_temp_point_list.empty()){
        point_list.clear();
        _index = 0;
        return;
    }
    
        
    for (auto& waypoint : point_list){
        hash_table.insert({waypoint.longitude, waypoint.latitude});
    }
    int waypoint_num = 0;
    for (auto& waypoint : _temp_point_list){
        waypoint_num++;
        auto result = hash_table.insert({waypoint.longitude, waypoint.latitude});
        if (result.second && _index >= waypoint_num){// 回到第一个不同的点开始跟踪
            _index = waypoint_num;
            break;
        }    
    }    
    point_list = std::move(_temp_point_list);  
}

std::pair<int, int> Point_Nav::get_thruster_pwm(void) const {
    return {_left_thruster_pwm, _right_thruster_pwm};
}

void Point_Nav::stop(void){
    _left_thruster_pwm = THRUSTER_MID_PWM;
    _right_thruster_pwm = THRUSTER_MID_PWM;
}

void Point_Nav::setPidParams(ros::NodeHandle& nh) {
    // 角度PID参数
    nh.getParam("point_nav/angle_kp", angle_pid.kp);
    nh.getParam("point_nav/angle_ki", angle_pid.ki);
    nh.getParam("point_nav/angle_kd", angle_pid.kd);
    nh.getParam("point_nav/maxIntegral", angle_pid.maxIntegral);
    nh.getParam("point_nav/maxOutput", angle_pid.maxOutput);

    // 速度PID参数
    nh.param<double>("point_nav/desired_speed", _desired_speed, 0.5);
    nh.getParam("point_nav/speed_kp", speed_pid.kp);
    nh.getParam("point_nav/speed_ki", speed_pid.ki);
    nh.getParam("point_nav/speed_kd", speed_pid.kd);
    nh.getParam("point_nav/maxIntegral", speed_pid.maxIntegral);
    nh.getParam("point_nav/maxOutput", speed_pid.maxOutput);
    
}
