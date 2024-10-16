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
#include "tcp/format_conversion.h"
#include "common.h"
#include "tcp/message_s.h"
#include "tcp/sendMessage.h"
#include <cmath>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

ros::Publisher controller_pub, debug_pub, viz_pub;
control::Motor thruster_msg;
double heading_angle, angle_pid_output, speed_pid_output;
sensor_msgs::Imu imu;
geometry_msgs::TwistStamped rtk_speed;
PID myPid;
ros::ServiceClient client;
Eigen::Vector3d desired_speed, course_speed, filtered_speed;

const int BUFFER_SIZE = 20;
std::deque<Eigen::Vector3d> speed_buffer;
Eigen::Matrix3d P, Q, R;

class ExtendedKalmanFilter
{
public:
    Eigen::Vector3d x; // 状态向量 [速度, 航向角, 角速度]
    Eigen::Matrix3d P; // 协方差矩阵

    ExtendedKalmanFilter()
    {
        x = Eigen::Vector3d::Zero();
        P = Eigen::Matrix3d::Identity() * 1000;
    }

    void predict(double dt)
    {
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 2) = -x(0) * sin(x(1)) * dt;
        F(1, 2) = dt;

        x(0) += x(2) * cos(x(1)) * dt;
        x(1) += x(2) * dt;

        P = F * P * F.transpose() + Q;
    }

    void update(const Eigen::Vector3d &z)
    {
        Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
        Eigen::Vector3d y = z - x;
        Eigen::Matrix3d S = H * P * H.transpose() + R;
        Eigen::Matrix3d K = P * H.transpose() * S.inverse();

        x = x + K * y;
        P = (Eigen::Matrix3d::Identity() - K * H) * P;
    }
};

ExtendedKalmanFilter ekf;

Eigen::Vector3d lowPassFilter(const Eigen::Vector3d &input, const Eigen::Vector3d &prev_output, double alpha)
{
    return alpha * input + (1 - alpha) * prev_output;
}

bool isOutlier(const Eigen::Vector3d &measurement, const Eigen::Vector3d &filtered, double threshold)
{
    return (measurement - filtered).norm() > threshold;
}

void adaptFilterParameters(const Eigen::Vector3d &innovation)
{
    double innovation_magnitude = innovation.norm();
    double alpha = 0.1; // 自适应率

    // 根据创新调整过程噪声
    Q = Q + alpha * (innovation * innovation.transpose() - Q);

    // 根据创新调整测量噪声
    R = R + alpha * (innovation * innovation.transpose() - R);
}

void Pid_Callback(const ros::TimerEvent &event)
{
    angle_pid_output = myPid.PID_AngleCalc(desired_speed(1), course_speed(1));
    speed_pid_output = myPid.PID_SpeedCalc(desired_speed(0), course_speed(0));

    ROS_INFO("speed_pid_output: %f, angle_pid_output: %f", speed_pid_output, angle_pid_output);

    double left_pwm = THRUSTER_MID_PWM + speed_pid_output - angle_pid_output;
    double right_pwm = THRUSTER_MID_PWM + speed_pid_output + angle_pid_output;

    thruster_msg.left_thruster_pwm = std::max(THRUSTER_MIN_PWM, std::min(THRUSTER_MAX_PWM, (int)left_pwm));
    thruster_msg.right_thruster_pwm = std::max(THRUSTER_MIN_PWM, std::min(THRUSTER_MAX_PWM, (int)right_pwm));

    controller_pub.publish(thruster_msg);
}

void rtkSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    rtk_speed = *msg;

    Eigen::Vector3d current_speed;
    current_speed(0) = sqrt(pow(rtk_speed.twist.linear.x, 2) + pow(rtk_speed.twist.linear.y, 2));
    current_speed(1) = std::atan2(rtk_speed.twist.linear.y, rtk_speed.twist.linear.x);
    current_speed(2) = rtk_speed.twist.angular.z;

    static Eigen::Vector3d prev_filtered_speed = Eigen::Vector3d::Zero();
    Eigen::Vector3d low_pass_filtered = lowPassFilter(current_speed, prev_filtered_speed, 0.2);
    prev_filtered_speed = low_pass_filtered;

    if (!isOutlier(low_pass_filtered, filtered_speed, 2.0))
    {
        ekf.predict(0.1); // 假设时间步长为0.1秒
        ekf.update(low_pass_filtered);
        filtered_speed = ekf.x;

        Eigen::Vector3d innovation = low_pass_filtered - filtered_speed;
        adaptFilterParameters(innovation);

        speed_buffer.push_back(filtered_speed);
        if (speed_buffer.size() > BUFFER_SIZE)
        {
            speed_buffer.pop_front();
        }

        Eigen::Vector3d avg_speed = Eigen::Vector3d::Zero();
        for (const auto &speed : speed_buffer)
        {
            avg_speed += speed;
        }
        avg_speed /= speed_buffer.size();

        course_speed = avg_speed;

        // 发布调试信息
        std_msgs::String debug_msg;
        debug_msg.data = "Filtered speed: " + std::to_string(filtered_speed(0)) +
                         ", Heading: " + std::to_string(filtered_speed(1)) +
                         ", Angular velocity: " + std::to_string(filtered_speed(2));
        debug_pub.publish(debug_msg);

        // 可视化
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "speed_vector";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), filtered_speed(1)));
        marker.scale.x = filtered_speed(0);
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        viz_pub.publish(marker);
    }
    else
    {
        ROS_WARN("Outlier detected, skipping update");
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    Eigen::Vector3d imu_speed;
    imu_speed(0) = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2));
    imu_speed(1) = std::atan2(msg->twist.twist.linear.y, msg->twist.twist.linear.x);
    imu_speed(2) = msg->twist.twist.angular.z;

    // 融合IMU和RTK数据
    filtered_speed = 0.7 * filtered_speed + 0.3 * imu_speed;

    heading_angle = tf2::getYaw(msg->pose.pose.orientation) * 180 / M_PI;
}

void Tcp_SubMessage_CallBack(const tcp::message_s::ConstPtr &msg)
{
    FormatConversionClass FormatConversion_;
    vector<std::string> sub_strings = msg->sub_strings;

    if (sub_strings[0] == "T" && sub_strings.size() == 5)
    {
        desired_speed(0) = stof(sub_strings[1]);
        myPid.angle_kp = stof(sub_strings[2]);
        myPid.angle_ki = stof(sub_strings[3]);
        myPid.angle_kd = stof(sub_strings[4]);
        ROS_INFO("Angle PID: %f, %f, %f", myPid.angle_kp, myPid.angle_ki, myPid.angle_kd);
    }
    if (sub_strings[0] == "T2" && sub_strings.size() == 5)
    {
        desired_speed(1) = stof(sub_strings[1]);
        myPid.speed_kp = stof(sub_strings[2]);
        myPid.speed_ki = stof(sub_strings[3]);
        myPid.speed_kd = stof(sub_strings[4]);
        ROS_INFO("Speed PID: %f, %f, %f", myPid.speed_kp, myPid.speed_ki, myPid.speed_kd);
    }
}

void TcpSend_Thread()
{
    tcp::sendMessage tcp_send_message;
    char tcp_sbuf[200];

    while (ros::ok())
    {
        snprintf(tcp_sbuf, sizeof(tcp_sbuf), "\nspeed_pid:%f,%f,%f,%d,%d,%f,%f,%f,%f,%f\n",
                 desired_speed(0), course_speed(0), desired_speed(0) - course_speed(0),
                 thruster_msg.left_thruster_pwm, thruster_msg.right_thruster_pwm,
                 course_speed(1), heading_angle, filtered_speed(0), filtered_speed(1), filtered_speed(2));

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
    debug_pub = nh.advertise<std_msgs::String>("/speed_debug", 1000);
    viz_pub = nh.advertise<visualization_msgs::Marker>("/speed_visualization", 1000);
    ros::Subscriber tcp_message_sub = nh.subscribe<tcp::message_s>("/tcp/data", 10000, Tcp_SubMessage_CallBack);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, odomCallback);
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

    Q = Eigen::Matrix3d::Identity() * 0.01;
    R = Eigen::Matrix3d::Identity() * 0.1;

    ROS_INFO("adjust_speed_pid_node running successfully.");

    std::thread TcpSend(TcpSend_Thread);

    ros::spin();

    TcpSend.join();

    return 0;
}