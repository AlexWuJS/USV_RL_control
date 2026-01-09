#include <iostream>
using namespace std;
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/String.h>
#include <string>
#include "adjust_pid/pid.h"
#include <tf2/utils.h>
#include <thruster_manager/Thruster.h>
#include "common.h"
#include "tcp/message_s.h"
#include "tcp/ship_info.h"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <sstream>


ros::Publisher controller_pub;
thruster_manager::Thruster thruster_msg;
double desired_angle, heading_angle, boat_distance, desired_position_x, desired_position_y, location_pid_output, angle_pid_output;
geometry_msgs::Point boat_position;                
PID PositionPid;
AnglePID AnglePid;
tcp::ship_info boat_status;
ros::Publisher tcp_tx_pub;

/**
  * @brief  从节点中读取上位机发送的参数等信息
  * @param  msg: ship_info类型的消息
  * @retval 无
  */
void Tcp_SubBoatMessage_CallBack(const tcp::ship_info::ConstPtr& msg)
{
    boat_status = *msg;
}

void Pid_Callback(const ros::TimerEvent &event)
{
    std_msgs::String tcp_sbuf;

    /* 到达目的地附近 */
    if (boat_distance < 1.5) // 1.5
    {
        /* 如果目标点在船身后,那么后退 */
        if ((desired_angle < AnglePID::map(heading_angle - 90)) && (desired_angle > AnglePID::map(heading_angle + 90)))
        {
            if (desired_angle != 0)
            {
                desired_angle = AnglePID::map(desired_angle - 180 * desired_angle/abs(desired_angle));
            }
            boat_distance = -boat_distance;
        }
    }
    /* 输出差速值 */
    location_pid_output = PositionPid.update(0, boat_distance); 
    angle_pid_output = AnglePid.update(desired_angle, heading_angle); 

    // if (boat_distance < 0.2 && boat_distance > -0.2)
    // {
    //     thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM;
    //     thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM;
    // }
    // else
    // {
        thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM - angle_pid_output * 0.5 + location_pid_output * 0.5;
        thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + angle_pid_output * 0.5 + location_pid_output * 0.5;
    // }


    /* 越界判断 */
    if (thruster_msg.left_thruster_pwm > THRUSTER_MAX_PWM)
        thruster_msg.left_thruster_pwm = THRUSTER_MAX_PWM;
    else if (thruster_msg.left_thruster_pwm < THRUSTER_MIN_PWM)
        thruster_msg.left_thruster_pwm = THRUSTER_MIN_PWM;

    if (thruster_msg.right_thruster_pwm > THRUSTER_MAX_PWM)
        thruster_msg.right_thruster_pwm = THRUSTER_MAX_PWM;
    else if (thruster_msg.right_thruster_pwm < THRUSTER_MIN_PWM)
        thruster_msg.right_thruster_pwm = THRUSTER_MIN_PWM;

    /* 发布pwm消息 */
    controller_pub.publish(thruster_msg);

    std::stringstream ss;

    ss << "location_pid:" << location_pid_output << "," 
       << boat_distance << "," 
       << thruster_msg.left_thruster_pwm << "," 
       << thruster_msg.right_thruster_pwm << ",\n";

    tcp_sbuf.data = ss.str();
    tcp_tx_pub.publish(tcp_sbuf);
}


/**
  * @brief  imu顺时针角度减少，逆时针角度增加
  * @param  无
  * @retval 无
  */
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    heading_angle = tf2::getYaw(msg->pose.pose.orientation)*180/M_PI;  // 当前船体的航向角

    boat_position.x = msg->pose.pose.position.x;
    boat_position.y = msg->pose.pose.position.y;

    /* 将经纬度转换为角度、位置误差 */
    desired_angle = atan2(desired_position_x - boat_position.x, desired_position_y - boat_position.y) * 180 / M_PI;
    boat_distance = sqrt(pow(desired_position_x - boat_position.x, 2) + pow(desired_position_y - boat_position.y, 2)); // 两点之间的距离
}

void Tcp_SubMessage_CallBack(const tcp::message_s::ConstPtr& msg)
{
    vector<std::string> sub_strings = msg->sub_strings;

    if (sub_strings[0] == "T" && sub_strings.size() == 6)      // T帧一共有5个数据
    {
        desired_position_x = stof(sub_strings[1]);
        desired_position_y = stof(sub_strings[2]);
        PositionPid.kp = stof(sub_strings[3]);
        PositionPid.ki = stof(sub_strings[4]);
        PositionPid.kd = stof(sub_strings[5]);
        ROS_INFO("pid:%f,%f,%f", PositionPid.kp,PositionPid.ki,PositionPid.kd);
    }
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "adjust_location_pid_node");

    ros::NodeHandle nh;

    controller_pub = nh.advertise<thruster_manager::Thruster>("/controller", 10);
    tcp_tx_pub = nh.advertise<std_msgs::String>("/tcp/tx_request", 10);

    ros::Subscriber tcp_message_sub = nh.subscribe<tcp::message_s>("/tcp/data", 10, Tcp_SubMessage_CallBack);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom_local", 1, odomCallback);
    ros::Subscriber tcp_boat_info_sub = nh.subscribe<tcp::ship_info>("/ship/ship_info", 10, Tcp_SubBoatMessage_CallBack);

    ros::Timer timer_pid = nh.createTimer(ros::Duration(0.1), Pid_Callback);
  
    //pid
    nh.getParam("/adjust_location_pid_node/angle_kp",AnglePid.kp);
    nh.getParam("/adjust_location_pid_node/angle_ki",AnglePid.ki);
    nh.getParam("/adjust_location_pid_node/angle_kd",AnglePid.kd);
    nh.getParam("/adjust_location_pid_node/location_kp",PositionPid.kp);
    nh.getParam("/adjust_location_pid_node/location_ki",PositionPid.ki);
    nh.getParam("/adjust_location_pid_node/location_kd",PositionPid.kd);
    nh.getParam("/adjust_location_pid_node/maxIntegral",AnglePid.maxIntegral);
    nh.getParam("/adjust_location_pid_node/maxOutput",AnglePid.maxOutput);
    nh.getParam("/adjust_location_pid_node/maxIntegral",PositionPid.maxIntegral);
    nh.getParam("/adjust_location_pid_node/maxOutput",PositionPid.maxOutput);

    ROS_INFO("adjust_location_pid_node run success.");

    ros::spin();

    return 0;
}  
