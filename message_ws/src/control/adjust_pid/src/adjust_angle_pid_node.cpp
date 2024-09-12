#include <iostream>
using namespace std;
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <thread>
#include <sys/socket.h>
#include <std_msgs/String.h>
#include <string>
// #include "tcp/Tcp.h"
#include "adjust_pid/pid.h"
#include <tf2/utils.h>
#include <control/Motor.h>
#include "tcp/format_conversion.h"
#include "common.h"

ros::Publisher controller_pub;
control::Motor thruster_msg;
double desired_angle, heading_angle, pid_output;
sensor_msgs::Imu imu;                        // 里程计信息
PID myPid;



void odomCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu = *msg;

    heading_angle = tf2::getYaw(imu.orientation);  // 当前船体的航向角
}

void Tcp_SubOriMessage_CallBack(const std_msgs::String::ConstPtr& msg)
{
    FormatConversionClass FormatConversion_;
    std::string str = msg->data;
    vector<string> string_values;

    switch (FormatConversion_.recognize_message(str))
    {
        case 'T':
            /* 接收键盘数据并记录 */
            string_values = FormatConversion_.get_value_from_string(str, ',');
            if (string_values.size() == 5)      // T帧一共有5个数据
            {
                desired_angle = stof(string_values[1]);
                myPid.kp_u = stof(string_values[2]);
                myPid.ki_u = stof(string_values[3]);
                myPid.kd_u = stof(string_values[4]);
                ROS_INFO("pid:%f,%f,%f", myPid.kp_u,myPid.ki_u,myPid.kd_u);
            }
        break;

        default:
            ROS_INFO("Message recognize error: Please check the message, maybe the message is not fit for the protocol!");
        break;
    }
}


int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "adjust_angle_pid_node");

    ros::NodeHandle nh;

    controller_pub = nh.advertise<control::Motor>("/controller", 1000);
    ros::Subscriber tcp_ori_message_sub = nh.subscribe<std_msgs::String>("/tcp/ori_message", 1000, Tcp_SubOriMessage_CallBack);
    ros::Subscriber odom_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, odomCallback);

    //pid
    nh.getParam("/adjust_angle_pid_node/kp_u",myPid.kp_u);
    nh.getParam("/adjust_angle_pid_node/ki_u",myPid.ki_u);
    nh.getParam("/adjust_angle_pid_node/kd_u",myPid.kd_u);
    nh.getParam("/adjust_angle_pid_node/maxIntegral",myPid.maxIntegral);
    nh.getParam("/adjust_angle_pid_node/maxOutput",myPid.maxOutput);

    ROS_INFO("adjust_angle_pid_node run success.");

    // 默认值1500
    thruster_msg.left_thruster_pwm = 1500;
    thruster_msg.right_thruster_pwm = 1500;

    ros::Rate r(0.1);
    while (ros::ok())
    {
        /* 输出差速值 */
        pid_output = myPid.PID_Calc(desired_angle, heading_angle); 
        thruster_msg.left_thruster_pwm += pid_output;
        thruster_msg.right_thruster_pwm -= pid_output;

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
        ROS_WARN("desired_angle:%f, heading_angle:%f    output:%f, maxOutput:%f,", desired_angle, heading_angle,pid_output, myPid.maxOutput);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}  