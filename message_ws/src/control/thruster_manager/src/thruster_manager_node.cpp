#include <clocale>
#include <cmath>

#include "ros/ros.h"

#include "thruster_manager/Thruster.h"
#include "tcp/joystick.h"
#include "tcp/ship_info.h"

// Joystick control tuning (PWM deltas around neutral)
#define JOYSTICK_PWM_SPAN               400     // |cmd|=1 -> +/- span around PWM_NEUTRAL
#define JOYSTICK_SIDE_PWM_SPAN          300     // side thruster assist span
#define JOYSTICK_DEADBAND_RAW           5       // |x|/|y| below this treated as 0 (raw units, [-100,100])

#define PWM_NEUTRAL                     1500
#define PWM_MIN                         1000
#define PWM_MAX                         2000

ros::Publisher controller_pub;
thruster_manager::Thruster auto_thruster;
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

void GetAutoThrusterPwm_CallBack(const thruster_manager::Thruster::ConstPtr& msg)
{
    auto_thruster = *msg;
}

void PublishMotorMsgTimer(const ros::TimerEvent &event)
{
    thruster_manager::Thruster motor_msg;
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
        // New protocol: K,x,y, where x/y in [-100,100] represent joystick position.
        // x: right +100, left -100; y: forward +100, backward -100
        int raw_x = boat_status.joystick.x;
        int raw_y = boat_status.joystick.y;

        if (std::abs(raw_x) < JOYSTICK_DEADBAND_RAW) raw_x = 0;
        if (std::abs(raw_y) < JOYSTICK_DEADBAND_RAW) raw_y = 0;

        double x = static_cast<double>(raw_x) / 100.0;
        double y = static_cast<double>(raw_y) / 100.0;

        // Clamp to unit circle to avoid diagonal overflow.
        const double mag = std::sqrt(x * x + y * y);
        if (mag > 1.0)
        {
            x /= mag;
            y /= mag;
        }

        // Differential-drive mix:
        // left_cmd  = y + x
        // right_cmd = y - x
        double left_cmd = y + x;
        double right_cmd = y - x;

        const double max_cmd = std::fmax(std::fabs(left_cmd), std::fabs(right_cmd));
        if (max_cmd > 1.0)
        {
            left_cmd /= max_cmd;
            right_cmd /= max_cmd;
        }

        motor_msg.left_thruster_pwm = PWM_NEUTRAL + static_cast<int>(left_cmd * JOYSTICK_PWM_SPAN);
        motor_msg.right_thruster_pwm = PWM_NEUTRAL + static_cast<int>(right_cmd * JOYSTICK_PWM_SPAN);

        // Side thrusters assist yaw more when forward speed is small.
        const double low_speed_factor = std::fmax(0.0, 1.0 - std::fabs(y));
        const int side_delta = static_cast<int>((-x) * JOYSTICK_SIDE_PWM_SPAN * low_speed_factor);
        motor_msg.left_sideThruster_pwm = PWM_NEUTRAL + side_delta;
        motor_msg.right_sideThruster_pwm = PWM_NEUTRAL + side_delta;
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
    ros::init(argc, argv, "thruster_manager_node");

    ros::NodeHandle nh;

    controller_pub = nh.advertise<thruster_manager::Thruster>("/controller", 10);
    ros::Subscriber boat_status_sub = nh.subscribe<tcp::ship_info>("/ship/ship_info", 10, Tcp_SubBoatStatus_CallBack);
    ros::Subscriber boat_auto_control_sub = nh.subscribe<thruster_manager::Thruster>("/auto_thruster_pwm", 10, GetAutoThrusterPwm_CallBack);

    ROS_INFO("thruster_manager_node run success.");
    auto_thruster.left_thruster_pwm = PWM_NEUTRAL;
    auto_thruster.left_sideThruster_pwm = PWM_NEUTRAL;
    auto_thruster.right_thruster_pwm = PWM_NEUTRAL;
    auto_thruster.right_sideThruster_pwm = PWM_NEUTRAL;

    ros::Timer motor_pub_timer = nh.createTimer(ros::Duration(0.1), PublishMotorMsgTimer);
    ros::spin();

    return 0;
}
