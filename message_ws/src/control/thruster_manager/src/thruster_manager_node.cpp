#include <clocale>
#include <cmath>

#include "ros/ros.h"

#include "thruster_manager/Thruster.h"
#include "tcp/joystick.h"
#include "tcp/ship_info.h"

// Joystick control tuning (PWM deltas around neutral)
#define JOYSTICK_PWM_SPAN               300     // 主推进器PWM变化范围 (归一化输入 1.0 对应 PWM 变化量)
#define JOYSTICK_DEADBAND_RAW           0.05    // 摇杆死区 (输入已归一化到 [-1, 1])，绝对值小于此数视为 0

#define PWM_NEUTRAL                     1500
#define PWM_MIN                         1200
#define PWM_MAX                         1800

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
    if (boat_status.is_auto_control)
    {
        motor_msg.left_thruster_pwm = auto_thruster.left_thruster_pwm;
        motor_msg.right_thruster_pwm = auto_thruster.right_thruster_pwm;
    }
    /* 摇杆控制模式 */
    else if (boat_status.is_keyboard_control)
    {
        // 输入已归一化到 [-1, 1]
        // X为左右(右+)，Y为前后(前+)
        double x = static_cast<double>(boat_status.joystick.x);
        double y = static_cast<double>(boat_status.joystick.y);

        // 死区处理 (防止回中抖动)
        if (std::abs(x) < JOYSTICK_DEADBAND_RAW) x = 0.0;
        if (std::abs(y) < JOYSTICK_DEADBAND_RAW) y = 0.0;

        // 安全钳制，避免上游异常值导致输出过大
        x = std::fmax(-1.0, std::fmin(1.0, x));
        y = std::fmax(-1.0, std::fmin(1.0, y));

        // 圆形限制 (防止对角线溢出)，确保向量模长不超过 1.0，保证各个方向最大速度一致
        const double mag = std::sqrt(x * x + y * y);
        if (mag > 1.0)
        {
            x /= mag;
            y /= mag;
        }

        // 差速混合算法 (Differential Drive Mixing)
        // 左电机 = 前进分量 + 转向分量
        // 右电机 = 前进分量 - 转向分量
        double left_cmd = y + x;
        double right_cmd = y - x;

        // 6. 输出饱和处理
        // 如果计算结果 > 1.0 (例如全速前进且全速右转)，按比例缩放回 1.0
        const double max_cmd = std::fmax(std::fabs(left_cmd), std::fabs(right_cmd));
        if (max_cmd > 1.0)
        {
            left_cmd /= max_cmd;
            right_cmd /= max_cmd;
        }

        motor_msg.left_thruster_pwm = PWM_NEUTRAL + static_cast<int>(left_cmd * JOYSTICK_PWM_SPAN);
        motor_msg.right_thruster_pwm = PWM_NEUTRAL + static_cast<int>(right_cmd * JOYSTICK_PWM_SPAN);
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
