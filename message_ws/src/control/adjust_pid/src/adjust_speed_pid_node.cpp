#include <iostream>
using namespace std;
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <thread>
#include <sys/socket.h>
#include <std_msgs/String.h>
#include <string>
#include "adjust_pid/pid.h"
#include <tf2/utils.h>
#include <control/Motor.h>
#include "tcp/format_conversion.h"
#include "common.h"
#include "tcp/message_s.h"
#include "tcp/sendMessage.h"
#include <cmath>
#include <Eigen/Eigen>
#include <geometry_msgs/TwistStamped.h>


ros::Publisher controller_pub;
control::Motor thruster_msg;
double heading_angle, angle_pid_output, speed_pid_output;
sensor_msgs::Imu imu;                        
geometry_msgs::TwistStamped rtk_speed;
PID myPid;
ros::ServiceClient client;
Eigen::Vector2d desired_speed, course_speed;

void Pid_Callback(const ros::TimerEvent &event)
{
    angle_pid_output = myPid.PID_AngleCalc(desired_speed(0), course_speed(0));     /* 输出差速值 */
    speed_pid_output = myPid.PID_SpeedCalc(desired_speed(1), course_speed(1)); 

    ROS_WARN("speed_pid_output:%f", speed_pid_output);
    thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM + speed_pid_output;
    thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + speed_pid_output;   
    // thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM - 0.5 * angle_pid_output + 0.5 * speed_pid_output;
    // thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + 0.5 * angle_pid_output + 0.5 * speed_pid_output;

    // thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM - angle_pid_output - THRUSTER_DEADAREA_PWM * (angle_pid_output/std::fabs(angle_pid_output));    //跳过死区
    // thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + angle_pid_output + THRUSTER_DEADAREA_PWM * (angle_pid_output/std::fabs(angle_pid_output));    //跳过死区

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
  * @brief  获取rtk的速度值
  * @param  无
  * @retval 无
  */
void rtkSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    rtk_speed = *msg;

    course_speed(0) = std::atan2(rtk_speed.twist.linear.y, rtk_speed.twist.linear.x)*180/M_PI;
    course_speed(1) = sqrt(pow(rtk_speed.twist.linear.x, 2) + pow(rtk_speed.twist.linear.y, 2));
}

/**
  * @brief  imu顺时针角度减少，逆时针角度增加
  * @param  无
  * @retval 无
  */
void odomCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu = *msg;

    heading_angle = tf2::getYaw(imu.orientation)*180/M_PI;  // 当前船体的航向角
}

void Tcp_SubMessage_CallBack(const tcp::message_s::ConstPtr& msg)
{
    FormatConversionClass FormatConversion_;
    vector<std::string> sub_strings = msg->sub_strings;

    if (sub_strings[0] == "T" && sub_strings.size() == 5)      // T帧一共有5个数据
    {
            desired_speed(0) = stof(sub_strings[1]);
            myPid.angle_kp = stof(sub_strings[2]);
            myPid.angle_ki = stof(sub_strings[3]);
            myPid.angle_kd = stof(sub_strings[4]);
            ROS_INFO("pid:%f,%f,%f", myPid.angle_kp,myPid.angle_ki,myPid.angle_kd);
    }
    if (sub_strings[0] == "T2" && sub_strings.size() == 5)      // T2帧一共有5个数据
    {
            desired_speed(1) = stof(sub_strings[1]);
            myPid.speed_kp = stof(sub_strings[2]);
            myPid.speed_ki = stof(sub_strings[3]);
            myPid.speed_kd = stof(sub_strings[4]);
            ROS_INFO("pid:%f,%f,%f", myPid.speed_kp,myPid.speed_ki,myPid.speed_kd);
    }
}

void TcpSend_Thread()
{
    tcp::sendMessage tcp_send_message;
    char tcp_sbuf[100];

    while (true)
    {
        sprintf(tcp_sbuf, "\nspeed_pid:%f,%f,%f,%d,%d,%f,%f\n",  desired_speed(1), course_speed(1),desired_speed(1)-course_speed(1), thruster_msg.left_thruster_pwm, thruster_msg.right_thruster_pwm, course_speed(0), heading_angle);
          
        tcp_send_message.request.str = tcp_sbuf;

        client.call(tcp_send_message);   
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));    // 在线程中延时 0.1 秒       
    }
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "adjust_speed_pid");

    ros::NodeHandle nh;

    controller_pub = nh.advertise<control::Motor>("/controller", 1000);
    ros::Subscriber tcp_message_sub = nh.subscribe<tcp::message_s>("/tcp/data", 10000, Tcp_SubMessage_CallBack);
    ros::Subscriber odom_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, odomCallback);
    ros::Subscriber rtk_speed_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vel", 1000, rtkSpeedCallback);
    client = nh.serviceClient<tcp::sendMessage>("/tcp/communication");

    ros::Timer timer_pid = nh.createTimer(ros::Duration(0.1), Pid_Callback);
  
    nh.getParam("/adjust_speed_pid/angle_kp",myPid.angle_kp);
    nh.getParam("/adjust_speed_pid/angle_ki",myPid.angle_ki);
    nh.getParam("/adjust_speed_pid/angle_kd",myPid.angle_kd);
    nh.getParam("/adjust_speed_pid/speed_kp",myPid.speed_kp);
    nh.getParam("/adjust_speed_pid/speed_ki",myPid.speed_ki);
    nh.getParam("/adjust_speed_pid/speed_kd",myPid.speed_kd);
    nh.getParam("/adjust_speed_pid/maxIntegral",myPid.maxIntegral);
    nh.getParam("/adjust_speed_pid/maxOutput",myPid.maxOutput);

    ROS_INFO("adjust_speed_pid_node run success.");

    // 创建线程，传入线程函数
    std::thread TcpSend(TcpSend_Thread); 

    ros::spin();

    // 等待线程结束
    TcpSend.join();

    return 0;
}  