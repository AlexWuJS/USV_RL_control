#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/String.h>
#include <string>
#include "adjust_pid/pid.h"
#include <tf2/utils.h>
#include <thruster_manager/Thruster.h>
#include "common.h"
#include "tcp/message_s.h"
#include <cmath>
#include <geometry_msgs/TwistStamped.h> 
#include <logger/logger.h>


ros::Publisher controller_pub;
ros::Publisher tcp_tx_pub;
thruster_manager::Thruster thruster_msg;
double heading_angle, angle_pid_output, speed_pid_output;
sensor_msgs::Imu imu;
geometry_msgs::TwistStamped rtk_speed;
Eigen::Vector2d desired_speed, course_speed, unfilter_speed;
PID SpeedPid;
AnglePID AnglePid;

void Pid_Callback(const ros::TimerEvent &event)
{
    speed_pid_output = SpeedPid.update(desired_speed(1), course_speed(1));

    thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM + speed_pid_output ;
    thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + speed_pid_output ;

    // 越界判断
    thruster_msg.left_thruster_pwm = std::max(THRUSTER_MIN_PWM, std::min(THRUSTER_MAX_PWM, thruster_msg.left_thruster_pwm));
    thruster_msg.right_thruster_pwm = std::max(THRUSTER_MIN_PWM, std::min(THRUSTER_MAX_PWM, thruster_msg.right_thruster_pwm));

    controller_pub.publish(thruster_msg);
}

void rtkSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    rtk_speed = *msg;

    Eigen::Vector2d current_speed;
    current_speed(0) = std::atan2(rtk_speed.twist.linear.x, rtk_speed.twist.linear.y) * 180 / M_PI;     // 用x/y，而不是y/x，完成从ENU到NED的转换
    current_speed(1) = std::sqrt(std::pow(rtk_speed.twist.linear.x, 2) + std::pow(rtk_speed.twist.linear.y, 2));

    unfilter_speed = current_speed;
    course_speed = current_speed;
}

void odomCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu = *msg;
    heading_angle = tf2::getYaw(imu.orientation) * 180 / M_PI;
}

void Tcp_SubMessage_CallBack(const tcp::message_s::ConstPtr &msg)
{
    std::vector<std::string> sub_strings = msg->sub_strings;

    if (sub_strings[0] == "T2" && sub_strings.size() == 5)
    {
        desired_speed(1) = stof(sub_strings[1]);
        SpeedPid.kp = stof(sub_strings[2]);
        SpeedPid.ki = stof(sub_strings[3]);
        SpeedPid.kd = stof(sub_strings[4]);
        ROS_INFO("pid:%f,%f,%f", SpeedPid.kp, SpeedPid.ki, SpeedPid.kd);
    }
}

void TcpSendTimerCallback(const ros::TimerEvent &)
{
    std_msgs::String tcp_msg;
    char tcp_sbuf[200];

    snprintf(tcp_sbuf, sizeof(tcp_sbuf), "\nspeed_pid:%f,%f,%f,%d,%d\n",
             desired_speed(1), course_speed(1), unfilter_speed(1),
             thruster_msg.left_thruster_pwm, thruster_msg.right_thruster_pwm);

    tcp_msg.data = tcp_sbuf;
    tcp_tx_pub.publish(tcp_msg);
    LOG_INFO("desired_speed:{}, current_speed:{}, kp ki kd:{} {} {},", desired_speed(1), course_speed(1), SpeedPid.kp, SpeedPid.ki, SpeedPid.kd);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "adjust_speed_pid");

    ros::NodeHandle nh;

    controller_pub = nh.advertise<thruster_manager::Thruster>("/auto_thruster_pwm", 10);
    tcp_tx_pub = nh.advertise<std_msgs::String>("/tcp/tx_request", 10);
    ros::Subscriber tcp_message_sub = nh.subscribe<tcp::message_s>("/tcp/data", 10, Tcp_SubMessage_CallBack);
    ros::Subscriber odom_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, odomCallback);
    ros::Subscriber rtk_speed_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vel", 10, rtkSpeedCallback);

    ros::Timer timer_pid = nh.createTimer(ros::Duration(0.1), Pid_Callback);
    ros::Timer timer_tcp_send = nh.createTimer(ros::Duration(0.1), TcpSendTimerCallback);

    nh.getParam("/adjust_speed_pid/speed_kp", SpeedPid.kp);
    nh.getParam("/adjust_speed_pid/speed_ki", SpeedPid.ki);
    nh.getParam("/adjust_speed_pid/speed_kd", SpeedPid.kd);
    nh.getParam("/adjust_speed_pid/maxIntegral", AnglePid.maxIntegral);
    nh.getParam("/adjust_speed_pid/maxOutput", AnglePid.maxOutput);
    nh.getParam("/adjust_speed_pid/maxIntegral", SpeedPid.maxIntegral);
    nh.getParam("/adjust_speed_pid/maxOutput", SpeedPid.maxOutput);

    ROS_INFO("adjust_speed_value_pid_node run success.");

    ros::spin();

    return 0;
}
