#include <clocale>

#include "ros/ros.h"

#include "control/Motor.h"
#include "tcp/keyboard.h"
#include "tcp/ship_info.h"

#define KEYBOARD_ADD_PWM                400     // 前进、后退改变的PWM
#define KEYBOARD_LEFT_DIFF_MPWM         600     // 左转主推进器的PWM差值,范围：0~1000
#define KEYBOARD_LEFT_ADD_SMPWM         300     // 左转侧推进器改变的PWM
#define KEYBOARD_RIGHT_DIFF_MPWM        600     // 右转主推进器的PWM差值,范围：0~1000
#define KEYBOARD_RIGHT_ADD_SMPWM        300     // 右转侧推进器改变的PWM

#define PWM_NEUTRAL                     1500
#define PWM_MIN                         1000
#define PWM_MAX                         2000

ros::Publisher controller_pub;
control::Motor auto_thruster;
tcp::ship_info boat_status;

static int clamp_pwm(int value)
{
    if (value < PWM_MIN) return PWM_MIN;
    if (value > PWM_MAX) return PWM_MAX;
    return value;
}

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

void PublishMotorMsgTimer(const ros::TimerEvent &event)
{
    control::Motor motor_msg;
    motor_msg.left_thruster_pwm = PWM_NEUTRAL;
    motor_msg.left_sideThruster_pwm = PWM_NEUTRAL;
    motor_msg.right_thruster_pwm = PWM_NEUTRAL;
    motor_msg.right_sideThruster_pwm = PWM_NEUTRAL;

    /* 自主巡航模式 */
    if (boat_status.is_auto_control || boat_status.boat_is_smart_remote_mode)
    {
        motor_msg.left_thruster_pwm = auto_thruster.left_thruster_pwm;
        motor_msg.right_thruster_pwm = auto_thruster.right_thruster_pwm;
    }
    /* 键盘控制模式 */
    else if (boat_status.is_keyboard_control)
    {
        const tcp::keyboard& myKeyboard = boat_status.keyboard;
        if (myKeyboard.W == 1)      // 前进
        {
            motor_msg.left_thruster_pwm += KEYBOARD_ADD_PWM;
            motor_msg.right_thruster_pwm += KEYBOARD_ADD_PWM;
        }
        if (myKeyboard.S == 1)      // 后退
        {
            motor_msg.left_thruster_pwm -= KEYBOARD_ADD_PWM;
            motor_msg.right_thruster_pwm -= KEYBOARD_ADD_PWM;
        }
        if (myKeyboard.A == 1)      // 左转
        {
            motor_msg.left_thruster_pwm -= (KEYBOARD_LEFT_DIFF_MPWM/2);
            motor_msg.right_thruster_pwm += (KEYBOARD_LEFT_DIFF_MPWM/2);
            motor_msg.left_sideThruster_pwm += KEYBOARD_LEFT_ADD_SMPWM;
            motor_msg.right_sideThruster_pwm += KEYBOARD_LEFT_ADD_SMPWM;
        }
        if (myKeyboard.D == 1)      // 右转
        {
            motor_msg.left_thruster_pwm -= (KEYBOARD_RIGHT_DIFF_MPWM/2);
            motor_msg.right_thruster_pwm += (KEYBOARD_RIGHT_DIFF_MPWM/2);
            motor_msg.left_sideThruster_pwm -= KEYBOARD_RIGHT_ADD_SMPWM;
            motor_msg.right_sideThruster_pwm -= KEYBOARD_RIGHT_ADD_SMPWM;
        }
    }

    motor_msg.left_thruster_pwm = clamp_pwm(motor_msg.left_thruster_pwm);
    motor_msg.left_sideThruster_pwm = clamp_pwm(motor_msg.left_sideThruster_pwm);
    motor_msg.right_thruster_pwm = clamp_pwm(motor_msg.right_thruster_pwm);
    motor_msg.right_sideThruster_pwm = clamp_pwm(motor_msg.right_sideThruster_pwm);

    controller_pub.publish(motor_msg);
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "controller_node");

    ros::NodeHandle nh;

    controller_pub = nh.advertise<control::Motor>("/controller", 10);
    ros::Subscriber boat_status_sub = nh.subscribe<tcp::ship_info>("/ship/ship_info", 10, Tcp_SubBoatStatus_CallBack);
    ros::Subscriber boat_auto_control_sub = nh.subscribe<control::Motor>("/auto_thruster_pwm", 10, GetAutoThrusterPwm_CallBack);

    ROS_INFO("controller_node run success.");
    auto_thruster.left_thruster_pwm = PWM_NEUTRAL;
    auto_thruster.left_sideThruster_pwm = PWM_NEUTRAL;
    auto_thruster.right_thruster_pwm = PWM_NEUTRAL;
    auto_thruster.right_sideThruster_pwm = PWM_NEUTRAL;

    ros::Timer motor_pub_timer = nh.createTimer(ros::Duration(0.1), PublishMotorMsgTimer);
    ros::spin();

    return 0;
}
