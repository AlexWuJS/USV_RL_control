#ifndef SHIP_NODE_H
#define SHIP_NODE_H


//C语言相关头文件
#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>


//ROS相关头文件
#include <ros/ros.h>    
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#define PI 3.1415926


class Ship
{
    public:
        Ship();//构造函数
        ~Ship();//析构函数
    private:

        

        //速度消息回调函数
        void cmdVelCallback(const nav_msgs::Path::ConstPtr& msg);

        //订阅器定义
        ros::Subscriber point_sub_;



};



#endif