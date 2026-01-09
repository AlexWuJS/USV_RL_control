#include <iostream>
using namespace std;
#include "ros/ros.h"
#include "usart/USART1.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <string>
#include <thruster_manager/Motor.h>
#include "string.h"

#define USART1_PORT		(char* )"/dev/32port"

double yaw, pitch, roll;
static char USART1_rbuf[100];

ros::Publisher mcu_pub, toMcu_pub;
thruster_manager::Motor motor_msg;

void SendToStm32_CallBack(const ros::TimerEvent &event)
{
    char USART1_sbuf[100];
    std_msgs::String toMcu;

    /*--- Send motor pwm to stm32 ---*/
    sprintf(USART1_sbuf,"M,%d,%d,%d,%d,", motor_msg.left_thruster_pwm, motor_msg.right_thruster_pwm, motor_msg.left_sideThruster_pwm, motor_msg.right_sideThruster_pwm);
    USART1_SendString(USART1_sbuf);
    /* --- Publish data from nx to mcu  ---*/ 
    toMcu.data = std::string(USART1_sbuf);
    toMcu_pub.publish(toMcu);
}

/**
  * @brief  每隔0.1s，从stm32中读取一帧信息
  * @param  无
  * @retval 无
  */
void Usart_GetMcuMessage(const ros::TimerEvent &event)
{
    /*--- Get mcu message from stm32 ---*/ 
    memset(USART1_rbuf, '\0', sizeof(USART1_rbuf));     // 接收前先清0，免得多出的数据没有被覆盖导致错误。
    USART_ReadString(USART1_rbuf, 100);
}


/**
  * @brief  从节点处获取PWM值，并发送给STM32
  * @param  无
  * @retval 无
  */
void Get_ThrusterPwm_CallBack(const thruster_manager::Motor::ConstPtr& msg)
{
    motor_msg = *msg;   // 得到pwm值
}

void PubMcuMessage_CallBack(const ros::TimerEvent &event)
{
    /* --- Publish message from mcu ---*/
    static std_msgs::String mcu_message;

    if (strcmp(USART1_rbuf, "") != 0)      // 如果数据不为空，更新数据
    {
        mcu_message.data.clear();
        mcu_message.data = std::string(USART1_rbuf);
    }
    mcu_pub.publish(mcu_message);
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "usart");

    ros::NodeHandle nh;

    mcu_pub = nh.advertise<std_msgs::String>("/usart/data", 100);
    toMcu_pub = nh.advertise<std_msgs::String>("/usart/toMcu", 100);
    ros::Subscriber controller_sub = nh.subscribe<thruster_manager::Motor>("/controller", 1, Get_ThrusterPwm_CallBack);

    /*--- USART1 INIT ------------------------------------------- */
    char* device_usart1 = USART1_PORT;
    while (-1 == USART1_Init(device_usart1))
    {
        // ROS_ERROR("USART1 init failed, trying connect again...");
        sleep(1);
    }
    ROS_INFO("usart1 node init success.");
    /*--- end of USART1 INIT ------------------------------------ */

    /* --- ROS send mcu message ------------------------------------------------------------------*/
    ros::Timer timer_get_mcu = nh.createTimer(ros::Duration(0.1), Usart_GetMcuMessage);
    ros::Timer timer_pub_mcu = nh.createTimer(ros::Duration(0.1), PubMcuMessage_CallBack);
    ros::Timer timer_send_to_mcu = nh.createTimer(ros::Duration(0.05), SendToStm32_CallBack);

    ros::spin();

    return 0;
}
