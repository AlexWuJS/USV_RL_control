#ifndef FORMAT_CONVERSION_CLASS_H
#define FORMAT_CONVERSION_CLASS_H

#include "ros/ros.h"
#include "tcp/ship_info.h"
#include <string>
#include <vector>
#include <sensor_msgs/NavSatFix.h>
using namespace std;

/* 键盘控制结构体 */
typedef struct {
    bool W;
    bool A;
    bool S;
    bool D;
}KeyboardStructure;

class Point
{
public:
    Point() = default;
    Point(float longitude, float latitude);

    float m_longitude;	/* 经度 */
    float m_latitude;	/* 纬度 */
};

class FormatConversionClass {
public:
    FormatConversionClass();
    int SplitString(string s, vector<string>& sub_strings);
    std::vector<std::string> get_value_from_string(std::string s, const char pattern);
    char recognize_message(std::string s);

    int handle_header_S(std::string s);
    int handle_header_P(std::string points_s, std::vector<sensor_msgs::NavSatFix> &point_list);
    int handle_header_K(std::string s);

    bool push_is_open(void);
    bool message_is_recieving(void);
    bool tcp_is_connecting(void);
    char control_mode(void);
    bool boat_keep_nav(void);
    std::string handle_R_string(void);
    KeyboardStructure keyboard(void);
    float get_voltage_left_battery(void);
    float get_voltage_right_battery(void);

    bool is_auto_control;
    bool is_keyboard_control;

private:
    bool m_push_is_open;
    bool m_is_recieving;
    bool m_is_connecting;
    char m_control_mode;
    bool m_boat_keep_nav;
    float m_voltage_left_battery;
    float m_voltage_right_battery;
    KeyboardStructure m_keyboard;
    std::string m_valid_headers;
};

#endif // FORMAT_CONVERSION_CLASS_H
