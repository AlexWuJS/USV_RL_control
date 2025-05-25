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


class FormatConversionClass {
private:
    FormatConversionClass():is_connecting(true) {}
    FormatConversionClass(const FormatConversionClass&) = delete;
    FormatConversionClass& operator=(const FormatConversionClass&) = delete;

    static FormatConversionClass singleton;
    static const std::string _valid_headers;    // 协议中所有的合法帧头
public:
    static FormatConversionClass& get_singleton();
    
    int SplitString(string s, vector<string>& sub_strings);
    std::vector<std::string> get_value_from_string(std::string s, const char pattern);
    char recognize_message(std::string s);

    int handle_header_S(std::string s);
    int handle_header_P(std::string points_s, std::vector<sensor_msgs::NavSatFix> &point_list);
    int handle_header_K(std::string s);

    bool is_auto_control;
    bool is_keyboard_control;
    bool push_is_open;
    bool is_recieving;
    bool is_connecting;
    char control_mode;
    bool boat_keep_nav;
    float voltage_left_battery;
    float voltage_right_battery;
    KeyboardStructure keyboard;

};

#endif // FORMAT_CONVERSION_CLASS_H
