#include <iostream>
using namespace std;
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <string>
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
ros::ServiceClient client;
control::Motor my_motor_msg, auto_thruster;
tcp::ship_info::ConstPtr ship_information_ptr;
tcp::ship_info boat_status;

/**
  * @brief  从节点中读取上位机发送的参数等信息
  * @param  msg: ship_info类型的消息
  * @retval 无
  */
void Tcp_SubBoatStatus_CallBack(const tcp::ship_info::ConstPtr& msg)
{
    boat_status = *msg;
}

void GetAutoThrusterPwm_CallBack(const control::Motor::ConstPtr& msg)
{
    auto_thruster = *msg;
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "controller_node");

    ros::NodeHandle nh;

    controller_pub = nh.advertise<control::Motor>("/controller", 1000);
    ros::Subscriber boat_status_sub = nh.subscribe<tcp::ship_info>("/ship/ship_info", 1000, Tcp_SubBoatStatus_CallBack);
    // ros::Subscriber boat_postion_control_sub = nh.subscribe<control::Motor>("/tcp_connection", 1000, &tcp::tcp_connection::Tcp_SubBoatPosition_CallBack, &tcp_connect);
    ros::Subscriber boat_auto_control_sub = nh.subscribe<control::Motor>("/auto_thruster_pwm", 100, GetAutoThrusterPwm_CallBack);
    ros::ServiceClient client = nh.serviceClient<tcp::tcp_connection>("/tcp_connection");

    ROS_INFO("controller_node run success.");
    while (ros::ok())
    {  
        /* 自主巡航模式 */
        if (boat_status.is_auto_control)
        {
            my_motor_msg.left_thruster_pwm = auto_thruster.left_thruster_pwm;
            my_motor_msg.right_thruster_pwm = auto_thruster.right_thruster_pwm;
        }

        /* 键盘控制模式 */
        else if (boat_status.is_keyboard_control)
        {
            tcp::keyboard myKeyboard = boat_status.keyboard;
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
        /* smart remote mode */
        if (boat_status.boat_is_smart_remote_mode && boat_status.is_auto_control == false)
        {
            if (boat_status.boat_is_active && !boat_status.boat_is_running)
            {
                ;
            }
        }
        /* 发布pwm消息 */
        controller_pub.publish(my_motor_msg);

        /* 发送完之后清0 */
        my_motor_msg.left_thruster_pwm = 1500;
        my_motor_msg.left_sideThruster_pwm = 1500;
        my_motor_msg.right_thruster_pwm = 1500;
        my_motor_msg.right_sideThruster_pwm = 1500;

        ros::spinOnce();
    }

    return 0;
}