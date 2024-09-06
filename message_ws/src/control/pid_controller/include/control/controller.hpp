#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <iostream>
using namespace std;
#include "ros/ros.h"
#include <string>
#include <vector>
#include "tcp/ship_info.h"
#include "tcp/keyboard.h"

class Controller {
public:
    Controller()
    { 
        // m_now_point = Point(0,0);
        // m_now_point_index = 0;
    }

    int set(tcp::ship_info ship_info)
    {
        m_ship_info = ship_info;

        return 0;
    }

    /* 是否为键盘控制模式 */
    bool is_keyborad(void)
    {
        if (m_ship_info.control_mode == '@')
            return true;
        else
            return false;
    }

    /* 是否为自动巡航模式 */
    bool is_auto(void)
    {
        if (m_ship_info.control_mode == '#')
            return true;
        else
            return false;
    }

    tcp::keyboard keyboard(void)
    {
        return m_ship_info.keyboard;
    }

    /* --- 点处理函数 ------------------------------ */
    // void clear_all_points(void)
    // {
    //     m_points.clear();
    // }
    // /* 返回当前跟踪点的vector索引 */
    // int now_point_index(void)
    // {
    //     return m_now_point_index;
    // }

    // /* 返回当前点 */
    // Point now_point(void)
    // {
    //     return m_points[m_now_point_index];
    // }

    // /* 开始下一个点 */
    // void follow_next_point(void)
    // {
    //     m_now_point_index++;
    // }

    // /* 打印当前需要输入的所有点 */
    // void show_all_points(void)
    // {
    //     for (vector<Point>::iterator it = m_points.begin(); it != m_points.end(); it++) {
    //         cout << "longtitude: " << it->m_longitude << endl;
    //         cout << "latitude: " << it->m_latitude << endl << endl;
    //     }
    // }

    // /* 添加需要跟踪的点 */
    // void add_point(float longitude, float latitude)
    // {
    //     Point point(longitude, latitude);

    //     m_points.push_back(point);
    // }

    // void add_points(vector<Point> points)
    // {
    //     for (vector<Point>::iterator it = points.begin(); it != points.end(); it++) {
    //         m_points.push_back(*it);
    //     }
    // }

private:
    tcp::ship_info m_ship_info;
    /* --- 点相关 ---- */
    // unsigned int m_now_point_index;
    // Point m_now_point;
    // vector<Point> m_points;


};

#endif
