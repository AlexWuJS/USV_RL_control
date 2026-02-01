#include <iostream>
using namespace std;
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/String.h>
#include <string>
#include "adjust_pid/pid.h"
#include <tf2/utils.h>
#include <geometry_msgs/TwistStamped.h> 
#include <thruster_manager/Thruster.h>
#include "common.h"
#include "tcp/message_s.h"
#include <cmath>
#include <logger/logger.h>
#include <Eigen/Dense>


ros::Publisher controller_pub;
ros::Publisher tcp_tx_pub;
thruster_manager::Thruster thruster_msg;
geometry_msgs::TwistStamped rtk_speed;
double desired_angle, heading_angle, pid_output, yaw_rate = 0.0;
sensor_msgs::Imu imu;                    
// PID myPid;
AnglePID AnglePid;
Eigen::Vector2d current_speed;


void Pid_Callback(const ros::TimerEvent &event)
{
    /* 输出差速值 */
    pid_output = AnglePid.update(desired_angle, heading_angle);
    // pid_output = myPid.PID_AngleCalc(desired_angle, heading_angle); 
    thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM - pid_output;
    thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + pid_output;
    // thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM - pid_output - THRUSTER_DEADAREA_PWM * (pid_output/std::fabs(pid_output));    //跳过死区
    // thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + pid_output + THRUSTER_DEADAREA_PWM * (pid_output/std::fabs(pid_output));    //跳过死区

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
}
/**
  * @brief  imu顺时针角度减少，逆时针角度增加
  * @param  无
  * @retval 无
  */
void odomCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu = *msg;

    heading_angle = tf2::getYaw(imu.orientation)*180/M_PI;  // 当前船体的航向角
    yaw_rate = imu.angular_velocity.z * 180.0 / M_PI;
}

void Tcp_SubMessage_CallBack(const tcp::message_s::ConstPtr& msg)
{
    vector<std::string> sub_strings = msg->sub_strings;

    if (sub_strings[0] == "T" && sub_strings.size() == 5)      // T帧一共有5个数据
    {
            desired_angle = stof(sub_strings[1]);
            AnglePid.kp = stof(sub_strings[2]);
            AnglePid.ki = stof(sub_strings[3]);
            AnglePid.kd = stof(sub_strings[4]);
            ROS_INFO("pid:%f,%f,%f", AnglePid.kp,AnglePid.ki,AnglePid.kd);
    }
}

void TcpSendTimerCallback(const ros::TimerEvent &)
{
    char tcp_sbuf[100];
    std_msgs::String tcp_msg;

    snprintf(tcp_sbuf, sizeof(tcp_sbuf), "\npid:%f,%f,%f,%d,%d,\n",
             desired_angle,
             heading_angle,
             desired_angle - heading_angle,
             thruster_msg.left_thruster_pwm,
             thruster_msg.right_thruster_pwm);
    // sprintf(tcp_sbuf, "angle_pid:%f,%f,\r\n", desired_angle, heading_angle);
    tcp_msg.data = tcp_sbuf;
    tcp_tx_pub.publish(tcp_msg);
}

void rtkSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    rtk_speed = *msg;

    current_speed(0) = std::atan2(rtk_speed.twist.linear.x, rtk_speed.twist.linear.y) * 180 / M_PI;     // 用x/y，而不是y/x，完成从ENU到NED的转换
    current_speed(1) = std::sqrt(std::pow(rtk_speed.twist.linear.x, 2) + std::pow(rtk_speed.twist.linear.y, 2));
}


void RecordDataCallback(const ros::TimerEvent &)
{
    LOG_INFO_TO("test",  "{:.8f},{:.8f},{:.8f},{:.8f},{},{},{},{},{}", 
        desired_angle,
        heading_angle,
        yaw_rate,
        current_speed(1),
        AnglePid.kp,
        AnglePid.ki,
        AnglePid.kd,
        thruster_msg.left_thruster_pwm,
        thruster_msg.right_thruster_pwm
    );
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "adjust_angle_pid_node");

    ros::NodeHandle nh;

    controller_pub = nh.advertise<thruster_manager::Thruster>("/auto_thruster_pwm", 10);
    tcp_tx_pub = nh.advertise<std_msgs::String>("/tcp/tx_request", 10);
    ros::Subscriber tcp_message_sub = nh.subscribe<tcp::message_s>("/tcp/data", 10, Tcp_SubMessage_CallBack);
    ros::Subscriber odom_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, odomCallback);

    ros::Timer timer_pid = nh.createTimer(ros::Duration(0.1), Pid_Callback);
    ros::Timer timer_tcp_send = nh.createTimer(ros::Duration(0.1), TcpSendTimerCallback);
    ros::Timer timer_record = nh.createTimer(ros::Duration(0.1), RecordDataCallback);
  
    //pid
    nh.getParam("/adjust_angle_pid_node/angle_kp", AnglePid.kp);
    nh.getParam("/adjust_angle_pid_node/angle_ki", AnglePid.ki);
    nh.getParam("/adjust_angle_pid_node/angle_kd", AnglePid.kd);
    nh.getParam("/adjust_angle_pid_node/maxIntegral",AnglePid.maxIntegral);
    nh.getParam("/adjust_angle_pid_node/maxOutput",AnglePid.maxOutput);

    ROS_INFO("adjust_angle_pid_node run success.");

    ros::spin();

    return 0;
}  
