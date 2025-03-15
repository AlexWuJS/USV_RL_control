#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <sensor_msgs/NavSatFix.h>

class Point_Nav{
public:
    vector<sensor_msgs::NavSatFix> point_list;

    Point_Nav() = default;
    void run(void);
    void update_recv_waypoints(void);
    bool recv_first_waypoint(sensor_msgs::NavSatFix waypoint) const;
    bool recv_last_waypoint(sensor_msgs::NavSatFix waypoint) const;
    void add_waypoint(sensor_msgs::NavSatFix waypoint);
    bool begin_recv(sensor_msgs::NavSatFix waypoint);
    std::pair<int, int> get_thruster_pwm(void) const;
private:
    int _index;
    int _switch_dist;
    int _left_thruster_pwm, _right_thruster_pwm;
    static const double _desired_dist = 0;
    // static const sensor_msgs::NavSatFix virtual_first_waypoint, virtual_last_waypoint;

    vector<sensor_msgs::NavSatFix> _temp_point_list;

    double get_angle(sensor_msgs::NavSatFix self_point, sensor_msgs::NavSatFix waypoint);
    double get_distance(sensor_msgs::NavSatFix self_point, sensor_msgs::NavSatFix waypoint);
    void Point_Nav::update_point(double dist);
};
