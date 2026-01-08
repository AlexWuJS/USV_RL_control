#include "ros/ros.h"
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include "tcp/format_conversion.h"
#include "tcp/ship_info.h"
using namespace std;
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/NavSatFix.h>
#include <unordered_set>


// 定义静态成员 singleton
FormatConversionClass FormatConversionClass::singleton;
const std::string FormatConversionClass::_valid_headers = "SPRKMXT";

FormatConversionClass& FormatConversionClass::get_singleton() {
    return singleton;
}
/**
  * @brief  分割一个包含不同帧头的字符串。只查找一次，不允许存在相同帧头
  * @param  无
  * @retval -1: 字符串中不存在合法帧头
  */
 int FormatConversionClass::SplitString(string source, vector<string>& data_frames_string)
{
    data_frames_string.clear();

    if (_valid_headers.empty()) {
        return -1; // 错误：分隔符集合为空
    }

    // 将分隔符存储到 unordered_set 中以便快速查找
    std::unordered_set<char> delim_set(_valid_headers.begin(), _valid_headers.end());

    size_t start = 0;
    // 跳过开头的非分隔符字符
    while (start < source.length() && delim_set.find(source[start]) == delim_set.end()) {
        ++start;
    }

    // 从第一个分隔符开始分割
    while (start < source.length()) {
        // 找到下一个分隔符的位置
        size_t end = start + 1;
        while (end < source.length() && delim_set.find(source[end]) == delim_set.end()) {
            ++end;
        }

        // 提取从 start 到 end（不包括 end）的子字符串
        data_frames_string.push_back(source.substr(start, end - start));

        // 更新 start 到下一个分隔符位置
        start = end;
    }

    return 0; // 成功
    // size_t pos;
    // map<int, char> frame_header_positions;

    // /* 查找所有对应的符号 */
    // for (auto& header : _valid_headers){
    //     if ((pos = s.find(header)) != string::npos)
    //         frame_header_positions.insert(pair<int, char>(pos, header));        /* 记录帧头及对应的位置 */  
    // }
    // // for (std::string::iterator it = _valid_headers.begin(); it != _valid_headers.end(); it ++){
    // //     if ((pos = s.find(*it)) != string::npos)
    // //         frame_header_positions.insert(pair<int, char>(pos, *it));        /* 记录帧头及对应的位置 */  
    // // }
    // if (frame_header_positions.empty())    /* 如果没有找到对应的帧头 */
    // {
    //     return -1;
    // }
    // else    /* 根据key值，分割字符串*/
    // {
        
    //     for (map<int, char>::iterator it = frame_header_positions.begin(); it != frame_header_positions.end(); it++)
    //     {
    //         map<int, char>::iterator next_it = std::next(it);
    //         if (next_it != frame_header_positions.end())                                          // 如果该标识符不是最后一个标识符
    //             data_frames_string.push_back(s.substr(it->first, next_it->first - it->first));     // 将当前标识符与下一个标识符之间的字符作为字符串
    //         else                                                                            // 如果该标识符为最后一个标识符
    //             data_frames_string.push_back(s.substr(it->first));                                 // 将最后一个标识符到结尾添加到字符串
    //     }
    // }
    
    // return 0;
}

vector<string> FormatConversionClass::get_value_from_string(std::string s, const char pattern)
{
    vector<std::string> string_values;
    std::string string_value;
    std::istringstream iss(s);

    // split string
    while (getline(iss, string_value, pattern))     // 将iss字符串按逗号分割，存储在string_value中
    {
        /* 两个逗号间没有数据做的处理 */
        if (string_value == "")
            string_value = "0";
        string_values.push_back(string_value);
    }

    return string_values;
}

char FormatConversionClass::recognize_message(std::string s)
{
    /* 首先判断帧头、帧尾是否合法 */
    if (_valid_headers.find(s.front()) == std::string::npos)    /* 帧头不合法 */
    {
        ROS_INFO("recognize_message: The message tail '%c' is invalid!", s.front());
        ROS_INFO("    Entire message: %s", s.c_str());
        return '\0';
    }

    if (s.back() != ',')    /* 帧尾不合法 */
    {
        ROS_INFO("recognize_message: The message tail '%c' is invalid!", s.back());
        ROS_INFO("    Entire message: %s", s.c_str());
        return '\0';
    }

    return s.front();
}

/**
     * @brief  处理s报文格式的数据
     * @param  s: 接收的字符串
     * @retval 
     *     @arg: 0：返回成功
     *     @arg: -1：帧头错误
     */
int FormatConversionClass::handle_header_S(std::string s)
{
    vector<string> string_values;

    /* 首先判断帧头是否正确 */
    if (s.front() != 'S')    /* 帧头错误 */
    {
        ROS_INFO("Handle header error: header is not 'S'!");
        return -1;
    }
    
    string_values = get_value_from_string(s, ',');

    /* 接收对应的数据并记录 */
    if (string_values.size() != 6)      // S帧一共有6个数据
    {
        return -2;
    }
    else
    {
        
        if (string_values[1] == "1")  push_is_open = true;        /* 1为打开，0为关闭 */
        else if (string_values[1] == "0")  push_is_open = false;
        
        if (string_values[2] == "3")  is_recieving = true;        /* 3为打开，2为关闭 */
        else if (string_values[2] == "2")  is_recieving = false;
        
        if (string_values[3] == "q"||string_values[3] =="exit")  is_connecting = false;    /* q或exit为关闭 */
        else is_connecting = true;

        // control_mode = (char)string_values[4][0];  // 修正注释掉的代码，移除 m_ 前缀
        if (string_values[4] == "#") {is_auto_control = true;is_keyboard_control = false;}        /* #为自动控制，@为键盘控制 */
        else if (string_values[4] == "@") {is_keyboard_control = true;is_auto_control = false;}

        if (string_values[5] == "1")  boat_keep_nav = true;        /* 1为打开，0为关闭 */
        else if (string_values[5] == "0")  boat_keep_nav = false;             
    }

    return 0;
}


/**
 * @brief  处理P报文格式的数据，将接收的点存放到类中
 * @param  points_s: 接收的字符串
 * @retval
 *     @arg: 0：返回成功
 *     @arg: -1：帧头错误
 */
int FormatConversionClass::handle_header_P(std::string points_s, std::vector<sensor_msgs::NavSatFix> &point_list)
{
    vector<string> string_values;
    sensor_msgs::NavSatFix point;

    if (points_s.front() != 'P')    /* 帧头错误 */
    {
        ROS_INFO("Handle header error: header is not 'P'!");
        return -1;
    }

    string_values = get_value_from_string(points_s, ',');
    string_values.erase(string_values.begin());      // 去掉'P'帧头

    /* 将所有经纬度转为点集 */
    for (size_t i = 0; i < string_values.size(); i++)
    {
        if (i % 2 == 0)
            point.longitude = stof(string_values[i]);       // 经度
        
        if (i % 2 == 1)
        {
            point.latitude = stof(string_values[i]);                     // 纬度
            point_list.push_back(point);
        }
    }       

    return 0;
}

/**
     * @brief  处理K报文格式的数据，将接收的键盘操控值存放到类中
     * @param  s: 接收的字符串
     * @retval
     *     @arg: 0：返回成功
     *     @arg: -1：帧头错误
     */
int FormatConversionClass::handle_header_K(std::string s)
{
    vector<string> string_values;

    if (s.front() != 'K')    /* 帧头错误 */
    {
        ROS_INFO("Handle header error: header is not 'K'!");
        return -1;
    }

    /* 接收摇杆数据并记录: K,x,y,  (x/y in [-100,100]) */
    string_values = get_value_from_string(s, ',');
    if (string_values.size() < 3)      // K帧最少3个数据: K,x,y
    {
        return -2;
    }
    else
    {
        joystick.x = stoi(string_values[1]);
        joystick.y = stoi(string_values[2]);
    }

    return 0;
}
