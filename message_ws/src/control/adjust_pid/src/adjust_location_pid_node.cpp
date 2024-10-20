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

ros::Publisher controller_pub;
control::Motor thruster_msg;
double desired_angle, heading_angle, location_pid_output, angle_pid_output;
sensor_msgs::Imu imu;                    
PID myPid;
ros::ServiceClient client;


void Pid_Callback(const ros::TimerEvent &event)
{
    /* 将经纬度转换为角度、位置误差 */

    /* 输出差速值 */
    location_pid_output = myPid.PID_LocationCalc(0, heading_angle); 
    angle_pid_output = myPid.PID_AngleCalc(desired_angle, heading_angle); 
    // thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM - location_pid_output * 0.5 - angle_pid_output * 0.5;
    // thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + location_pid_output * 0.5 + angle_pid_output * 0.5;

    if (desired_angle -heading_angle > 0.5 || desired_angle -heading_angle < -0.5)
    {
        thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM - angle_pid_output;
        thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + angle_pid_output;
    }
    else
    {
        thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM + location_pid_output;
        thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + location_pid_output;
    }

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
}

void Tcp_SubMessage_CallBack(const tcp::message_s::ConstPtr& msg)
{
    FormatConversionClass FormatConversion_;
    vector<std::string> sub_strings = msg->sub_strings;

    if (sub_strings[0] == "T" && sub_strings.size() == 5)      // T帧一共有5个数据
    {
        desired_angle = stof(sub_strings[1]);
        myPid.location_kp = stof(sub_strings[2]);
        myPid.location_ki = stof(sub_strings[3]);
        myPid.location_kd = stof(sub_strings[4]);
        ROS_INFO("pid:%f,%f,%f", myPid.location_kp,myPid.location_ki,myPid.location_kd);
    }
}

void TcpSend_Thread()
{
    tcp::sendMessage tcp_send_message;
    char tcp_sbuf[100];

    while (true)
    {
        sprintf(tcp_sbuf, "\npid:%f,%f,%f,%d,%d,\n",  desired_angle, heading_angle,desired_angle-heading_angle, thruster_msg.left_thruster_pwm, thruster_msg.right_thruster_pwm);
        // sprintf(tcp_sbuf, "location_pid:%f,%f,\r\n", desired_angle, heading_angle);
          
        tcp_send_message.request.str = tcp_sbuf;

        client.call(tcp_send_message);   
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));    // 在线程中延时 0.1 秒       
    }
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "adjust_location_pid_node");

    ros::NodeHandle nh;

    controller_pub = nh.advertise<control::Motor>("/controller", 1000);
    ros::Subscriber tcp_message_sub = nh.subscribe<tcp::message_s>("/tcp/data", 10000, Tcp_SubMessage_CallBack);
    ros::Subscriber odom_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, odomCallback);

    client = nh.serviceClient<tcp::sendMessage>("/tcp/communication");

    ros::Timer timer_pid = nh.createTimer(ros::Duration(0.1), Pid_Callback);
  
    //pid
    nh.getParam("/adjust_location_pid_node/location_kp",myPid.location_kp);
    nh.getParam("/adjust_location_pid_node/location_ki",myPid.location_ki);
    nh.getParam("/adjust_location_pid_node/location_kd",myPid.location_kd);
    nh.getParam("/adjust_location_pid_node/maxIntegral",myPid.maxIntegral);
    nh.getParam("/adjust_location_pid_node/maxOutput",myPid.maxOutput);

    ROS_INFO("adjust_location_pid_node run success.");

    // 创建线程，传入线程函数
    std::thread TcpSend(TcpSend_Thread); 

    ros::spin();

    // 等待线程结束
    TcpSend.join();

    return 0;
}  