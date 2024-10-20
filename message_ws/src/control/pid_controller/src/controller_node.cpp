#include <iostream>
using namespace std;
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <string>
#include "control/controller.hpp"
#include "control/Motor.h"
#include "tcp/tcp_connection.h"
#include "tcp/ship_info.h"
#include "tcp/keyboard.h"

#define KEYBOARD_ADD_PWM                400     // 前进、后退改变的PWM
#define KEYBOARD_LEFT_DIFF_MPWM         600     // 左转主推进器的PWM差值,范围：0~1000
#define KEYBOARD_LEFT_ADD_SMPWM         300     // 左转侧推进器改变的PWM
#define KEYBOARD_RIGHT_DIFF_MPWM        600     // 右转主推进器的PWM差值,范围：0~1000
#define KEYBOARD_RIGHT_ADD_SMPWM        300     // 右转侧推进器改变的PWM


ros::Publisher controller_pub;
tcp::tcp_connection tcp_connect;
Controller myController;
ros::ServiceClient client;
control::Motor my_motor_msg;
tcp::ship_info::ConstPtr ship_information_ptr;

/**
  * @brief  从节点中读取上位机发送的参数等信息
  * @param  msg: ship_info类型的消息
  * @retval 无
  */
void Tcp_SubShipMessage_CallBack(const tcp::ship_info::ConstPtr& msg)
{
    ship_information_ptr = msg;
    myController.set(*ship_information_ptr);
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "controller_node");

    ros::NodeHandle nh;

    controller_pub = nh.advertise<control::Motor>("/controller", 1000);
    ros::Subscriber tcp_message_sub = nh.subscribe<tcp::ship_info>("/ship/ship_info", 1000, Tcp_SubShipMessage_CallBack);
    ros::ServiceClient client = nh.serviceClient<tcp::tcp_connection>("/tcp_connection");
    // ros::Subscriber listener = nh.subscribe<nav_msgs::Odometry>("/odom_local", 1000, GetAngle_CallBack);     // get eular angle from imu

    ROS_INFO("controller_node run success.");
    while (ros::ok())
    { 
  
        /* 自主巡航模式 */
        if (myController.is_auto() != false)
        {
            ;
        }

        /* 键盘控制模式 */
        if (myController.is_keyborad() != false)
        {
            tcp::keyboard myKeyboard;
            myKeyboard = myController.keyboard();
            if (myKeyboard.W == 1)      // 前进
            {
                my_motor_msg.left_thruster_pwm += KEYBOARD_ADD_PWM;
                my_motor_msg.right_thruster_pwm += KEYBOARD_ADD_PWM;
            }
            if (myKeyboard.S == 1)      // 后退
            {
                my_motor_msg.left_thruster_pwm -= KEYBOARD_ADD_PWM;
                my_motor_msg.right_thruster_pwm -= KEYBOARD_ADD_PWM;
            }
            if (myKeyboard.A == 1)      // 左转
            {
                my_motor_msg.left_thruster_pwm -= (KEYBOARD_LEFT_DIFF_MPWM/2);
                my_motor_msg.right_thruster_pwm += (KEYBOARD_LEFT_DIFF_MPWM/2);
                my_motor_msg.left_sideThruster_pwm += KEYBOARD_LEFT_ADD_SMPWM;
                my_motor_msg.right_sideThruster_pwm += KEYBOARD_LEFT_ADD_SMPWM;
            }
            if (myKeyboard.D == 1)      // 右转
            {
                my_motor_msg.left_thruster_pwm -= (KEYBOARD_RIGHT_DIFF_MPWM/2);
                my_motor_msg.right_thruster_pwm += (KEYBOARD_RIGHT_DIFF_MPWM/2);
                my_motor_msg.left_sideThruster_pwm -= KEYBOARD_RIGHT_ADD_SMPWM;
                my_motor_msg.right_sideThruster_pwm -= KEYBOARD_RIGHT_ADD_SMPWM;
            }
        }
        /* 发布pwm消息 */
        controller_pub.publish(my_motor_msg);

        cout << "keyboard" << my_motor_msg.left_thruster_pwm << " " << my_motor_msg.left_sideThruster_pwm << " " << my_motor_msg.right_thruster_pwm << " " << my_motor_msg.right_sideThruster_pwm << endl;

        /* 发送完之后清0 */
        my_motor_msg.left_thruster_pwm = 1500;
        my_motor_msg.left_sideThruster_pwm = 1500;
        my_motor_msg.right_thruster_pwm = 1500;
        my_motor_msg.right_sideThruster_pwm = 1500;

        ros::spinOnce();
    }

    return 0;
}