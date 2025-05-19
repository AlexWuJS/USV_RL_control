#pragma once

#include <string>
#include <vector>
#include <set>
#include <utility>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include "adjust_pid/pid.h"

// 定义电机中值常量
#define THRUSTER_MID_PWM 1500  // 推进器中间值PWM (通常是1500微秒)

class Point_Nav{
public:
    std::vector<sensor_msgs::NavSatFix> point_list;

    Point_Nav() = default;
    void run(sensor_msgs::NavSatFix current_position, double now_angle);
    void update_recv_waypoints(void);
    bool recv_first_waypoint(sensor_msgs::NavSatFix waypoint) const;
    bool recv_last_waypoint(sensor_msgs::NavSatFix waypoint) const;
    void add_waypoint(sensor_msgs::NavSatFix waypoint);
    void begin_recv(void);
    std::pair<int, int> get_thruster_pwm(void) const;
    
    // 添加设置PID参数的方法
    void setPidParams(ros::NodeHandle& nh);

private:
    int _index = 0; // 初始化为0，避免未初始化使用
    int _switch_dist = 4;       // 切换航点的距离,单位: m
    int _left_thruster_pwm = 1500, _right_thruster_pwm = 1500; // 初始化PWM值
    const double _desired_dist = 0;
    const double desired_angle = 0;
    static const sensor_msgs::NavSatFix virtual_start_waypoint, virtual_end_waypoint;

    std::vector<sensor_msgs::NavSatFix> _temp_point_list;
    
    // 添加PID控制器对象
    AnglePID angle_pid;
    PID position_pid;

    double get_angle(sensor_msgs::NavSatFix current_position, sensor_msgs::NavSatFix waypoint);
    double get_distance(sensor_msgs::NavSatFix current_position, sensor_msgs::NavSatFix waypoint);
    void update_point(double dist);
    void stop(void);
};
