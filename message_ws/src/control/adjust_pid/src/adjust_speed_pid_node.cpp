#include <iostream>
#include <deque>
#include <numeric>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <thread>
#include <sys/socket.h>
#include <std_msgs/String.h>
#include <string>
#include "adjust_pid/pid.h"
#include <tf2/utils.h>
#include <control/Motor.h>
#include "common.h"
#include "tcp/message_s.h"
#include "tcp/sendMessage.h"
#include <cmath>
#include <geometry_msgs/TwistStamped.h> 

ros::Publisher controller_pub;
control::Motor thruster_msg;
double heading_angle, angle_pid_output, speed_pid_output;
sensor_msgs::Imu imu;
geometry_msgs::TwistStamped rtk_speed;
PID myPid;
ros::ServiceClient client;
Eigen::Vector2d desired_speed, course_speed, filtered_speed, unfilter_speed;

const int BUFFER_SIZE = 10;
std::deque<Eigen::Vector2d> speed_buffer;
Eigen::Matrix2d P;

// 改进的低通滤波
Eigen::Vector2d lowPassFilter(const Eigen::Vector2d &input, const Eigen::Vector2d &prev_output, double alpha)
{
    return alpha * input + (1 - alpha) * prev_output;
}

// 改进的卡尔曼滤波
void updateKalmanFilter(const Eigen::Vector2d &measurement)
{
    static const double Q = 0.005; // 降低过程噪声
    static const double R = 0.2;   // 增加测量噪声
    static const Eigen::Matrix2d I = Eigen::Matrix2d::Identity();

    Eigen::Matrix2d F = Eigen::Matrix2d::Identity();
    filtered_speed = F * filtered_speed;
    P = F * P * F.transpose() + Q * I;

    Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d K = P * H.transpose() * (H * P * H.transpose() + R * I).inverse();
    filtered_speed = filtered_speed + K * (measurement - H * filtered_speed);
    P = (I - K * H) * P;
}

// 互补滤波
Eigen::Vector2d complementaryFilter(const Eigen::Vector2d &acc, const Eigen::Vector2d &gyro, double dt, double alpha)
{
    static Eigen::Vector2d angle = Eigen::Vector2d::Zero();
    angle = alpha * (angle + gyro * dt) + (1 - alpha) * acc;
    return angle;
}

// 平滑处理
Eigen::Vector2d smoothSpeed(const Eigen::Vector2d &input_speed)
{
    static std::deque<Eigen::Vector2d> speed_history;
    speed_history.push_back(input_speed);
    if (speed_history.size() > 5)
    {
        speed_history.pop_front();
    }

    Eigen::Vector2d smoothed_speed = Eigen::Vector2d::Zero();
    for (const auto &speed : speed_history)
    {
        smoothed_speed += speed;
    }
    return smoothed_speed / speed_history.size();
}

void Pid_Callback(const ros::TimerEvent &event)
{
    angle_pid_output = myPid.PID_AngleCalc(desired_speed(0), course_speed(0));
    speed_pid_output = myPid.PID_SpeedCalc(desired_speed(1), course_speed(1));

    ROS_WARN("speed_pid_output:%f", speed_pid_output);
    thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM + speed_pid_output ;
    thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + speed_pid_output ;

    // 越界判断
    thruster_msg.left_thruster_pwm = std::max(THRUSTER_MIN_PWM, std::min(THRUSTER_MAX_PWM, thruster_msg.left_thruster_pwm));
    thruster_msg.right_thruster_pwm = std::max(THRUSTER_MIN_PWM, std::min(THRUSTER_MAX_PWM, thruster_msg.right_thruster_pwm));

    controller_pub.publish(thruster_msg);
}

void rtkSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;

    rtk_speed = *msg;

    Eigen::Vector2d current_speed;
    current_speed(0) = std::atan2(rtk_speed.twist.linear.y, rtk_speed.twist.linear.x) * 180 / M_PI;
    unfilter_speed(1) = current_speed(1) = sqrt(pow(rtk_speed.twist.linear.x, 2) + pow(rtk_speed.twist.linear.y, 2));

    static Eigen::Vector2d prev_filtered_speed = Eigen::Vector2d::Zero();
    Eigen::Vector2d low_pass_filtered = lowPassFilter(current_speed, prev_filtered_speed, 0.1);
    prev_filtered_speed = low_pass_filtered;

    // 使用互补滤波
    static Eigen::Vector2d prev_speed = Eigen::Vector2d::Zero();
    Eigen::Vector2d gyro_speed = (current_speed - prev_speed) / dt;
    Eigen::Vector2d comp_filtered = complementaryFilter(low_pass_filtered, gyro_speed, dt, 0.98);
    prev_speed = current_speed;

    // 使用卡尔曼滤波
    updateKalmanFilter(comp_filtered);

    // 平滑处理
    Eigen::Vector2d smoothed_speed = smoothSpeed(filtered_speed);

    speed_buffer.push_back(smoothed_speed);
    if (speed_buffer.size() > BUFFER_SIZE)
    {
        speed_buffer.pop_front();
    }

    Eigen::Vector2d avg_speed = Eigen::Vector2d::Zero();
    for (const auto &speed : speed_buffer)
    {
        avg_speed += speed;
    }
    avg_speed /= speed_buffer.size();

    course_speed = avg_speed;
}

void odomCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu = *msg;
    heading_angle = tf2::getYaw(imu.orientation) * 180 / M_PI;
}

void Tcp_SubMessage_CallBack(const tcp::message_s::ConstPtr &msg)
{
    std::vector<std::string> sub_strings = msg->sub_strings;

    if (sub_strings[0] == "T" && sub_strings.size() == 5)
    {
        desired_speed(0) = stof(sub_strings[1]);
        myPid.angle_kp = stof(sub_strings[2]);
        myPid.angle_ki = stof(sub_strings[3]);
        myPid.angle_kd = stof(sub_strings[4]);
        ROS_INFO("pid:%f,%f,%f", myPid.angle_kp, myPid.angle_ki, myPid.angle_kd);
    }
    if (sub_strings[0] == "T2" && sub_strings.size() == 5)
    {
        desired_speed(1) = stof(sub_strings[1]);
        myPid.speed_kp = stof(sub_strings[2]);
        myPid.speed_ki = stof(sub_strings[3]);
        myPid.speed_kd = stof(sub_strings[4]);
        ROS_INFO("pid:%f,%f,%f", myPid.speed_kp, myPid.speed_ki, myPid.speed_kd);
    }
}

void TcpSend_Thread()
{
    tcp::sendMessage tcp_send_message;
    char tcp_sbuf[200];

    while (ros::ok())
    {
        snprintf(tcp_sbuf, sizeof(tcp_sbuf), "\nspeed_pid:%f,%f,%f,%d,%d,%f,%f\n",
                 desired_speed(1), course_speed(1), unfilter_speed(1),
                 thruster_msg.left_thruster_pwm, thruster_msg.right_thruster_pwm,
                 course_speed(0), heading_angle);

        tcp_send_message.request.str = tcp_sbuf;

        if (!client.call(tcp_send_message))
        {
            ROS_ERROR("Failed to send TCP message");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "adjust_speed_pid");

    ros::NodeHandle nh;

    controller_pub = nh.advertise<control::Motor>("/controller", 1000);
    ros::Subscriber tcp_message_sub = nh.subscribe<tcp::message_s>("/tcp/data", 10000, Tcp_SubMessage_CallBack);
    ros::Subscriber odom_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, odomCallback);
    ros::Subscriber rtk_speed_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vel", 1000, rtkSpeedCallback);
    client = nh.serviceClient<tcp::sendMessage>("/tcp/communication");

    ros::Timer timer_pid = nh.createTimer(ros::Duration(0.1), Pid_Callback);

    nh.getParam("/adjust_speed_pid/angle_kp", myPid.angle_kp);
    nh.getParam("/adjust_speed_pid/angle_ki", myPid.angle_ki);
    nh.getParam("/adjust_speed_pid/angle_kd", myPid.angle_kd);
    nh.getParam("/adjust_speed_pid/speed_kp", myPid.speed_kp);
    nh.getParam("/adjust_speed_pid/speed_ki", myPid.speed_ki);
    nh.getParam("/adjust_speed_pid/speed_kd", myPid.speed_kd);
    nh.getParam("/adjust_speed_pid/maxIntegral", myPid.maxIntegral);
    nh.getParam("/adjust_speed_pid/maxOutput", myPid.maxOutput);

    P = Eigen::Matrix2d::Identity() * 1000;

    ROS_INFO("adjust_speed_pid_node run success.");

    std::thread TcpSend(TcpSend_Thread);

    ros::spin();

    TcpSend.join();

    return 0;
}