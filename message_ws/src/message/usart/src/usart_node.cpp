#include <iostream>
using namespace std;
#include "ros/ros.h"
#include "usart/USART1.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <string>
#include <control/Motor.h>
#include "string.h"

#define USART1_PORT		(char* )"/dev/32port"

double yaw, pitch, roll;
static char* USART1_rbuf = new char(20);

ros::Publisher environment_pub;
control::Motor motor_msg;

void SendToStm32_CallBack(const ros::TimerEvent &event)
{
    char USART1_sbuf[100];

    /*--- Send motor pwm to stm32 ---*/
    sprintf(USART1_sbuf,"M,%d,%d,%d,%d,", motor_msg.left_thruster_pwm, motor_msg.right_thruster_pwm, motor_msg.LSideMotor_PWM, motor_msg.RSideMotor_PWM);
    USART1_SendString(USART1_sbuf); 
    ROS_INFO("Usart send to stm32:%s", USART1_sbuf);
}

/**
  * @brief  每隔0.1s，从stm32中读取一帧环境信息
  * @param  无
  * @retval 无
  */
void Usart_GetEnvironmentMessage(const ros::TimerEvent &event)
{
    /*--- Get environment message from stm32 ---*/ 
    memset(USART1_rbuf, '\0', sizeof(USART1_rbuf));     // 接收前先清0，免得多出的数据没有被覆盖导致错误。
    USART_ReadString(USART1_rbuf, 20);
}


/**
  * @brief  从节点处获取PWM值，并发送给STM32
  * @param  无
  * @retval 无
  */
void Get_MotorPwm_CallBack(const control::Motor::ConstPtr& msg)
{
    motor_msg = *msg;   // 得到pwm值

    /* --- Get eular angle ----------------------------------*/
    // tf::Quaternion quat;
    // tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void PubEnvironmentMessage_CallBack(const ros::TimerEvent &event)
{
    /* --- Publish environment message ---*/
    static std_msgs::String environment_message;

    if (strcmp(USART1_rbuf, "") != 0)      // 如果数据不为空，更新数据
    {
        environment_message.data = std::string(USART1_rbuf);
    }
    environment_pub.publish(environment_message);
    // ROS_INFO("Recieve info from Usart: %s\r\n", environment_message.data.c_str());
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "usart_node");

    ros::NodeHandle nh;

    environment_pub = nh.advertise<std_msgs::String>("/usart_message", 1000);
    ros::Subscriber controller_sub = nh.subscribe<control::Motor>("/controller", 1000, Get_MotorPwm_CallBack);

    /*--- USART1 INIT ------------------------------------------- */
    char* device_usart1 = USART1_PORT;
    while (-1 == USART1_Init(device_usart1))
    {
        // ROS_ERROR("USART1 init failed, trying connect again...");
        sleep(1);
    }
    ROS_INFO("usart1 node init success.");
    /*--- end of USART1 INIT ------------------------------------ */

    /* --- ROS send environmnet message ------------------------------------------------------------------*/
    ros::Timer timer_get_environment = nh.createTimer(ros::Duration(0.1), Usart_GetEnvironmentMessage);
    ros::Timer timer_pub_environment = nh.createTimer(ros::Duration(0.1), PubEnvironmentMessage_CallBack);
    ros::Timer timer_send_to_stm32 = nh.createTimer(ros::Duration(0.05), SendToStm32_CallBack);

    ros::spin();

    return 0;
}
