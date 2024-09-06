#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>


ros::Publisher twist_pub;
double theta = 0.0;
double vth = 0.0;
double linear_acceleration_x = 0.0;
double linear_acceleration_y = 0.0;
double angular_velocity_z = 0.0;
double linear_velocity_x = 0.0;
double linear_velocity_y = 0.0;
double g = 9.97;
ros::Time current_time;
ros::Time previous_time;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    //读取imu的角速度、线加速度信息
    //!!注意:imu实际安装时,Y轴负方向为船艏方向,X正方向为左向!!
    //此时在船体坐标系下,需要得到的x正方向为船体速度的前向,y正方向为船体的速度的左向
    angular_velocity_z = msg->angular_velocity.z;

    linear_acceleration_x = msg->linear_acceleration.y;
    linear_acceleration_x = -linear_acceleration_x;
    linear_acceleration_y = msg->linear_acceleration.x;
    ROS_INFO("original---X_acc: %f, Y_acc: %f", linear_acceleration_x, linear_acceleration_y);
    // 从IMU消息中获取四元数
    geometry_msgs::Quaternion quat = msg->orientation;

    // 将四元数转换为欧拉角
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // ROS_INFO("original--Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
    //根据IMU与实际船艏方向关系,进行RP转换
    double tfRPY = roll;
    roll = pitch;
    pitch = tfRPY;
    ROS_INFO("realRPY-Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);

    //补偿重力加速度在XY方向造成的分量
    linear_acceleration_x = linear_acceleration_x - (g * sin(pitch)* cos(roll));
    linear_acceleration_y = linear_acceleration_y - (g * sin(roll)* cos(pitch));
    ROS_INFO("NEW---X_acc: %f, Y_acc: %f", linear_acceleration_x, linear_acceleration_y);

    current_time = ros::Time::now();

    //积分计算线速度
    double dt = (current_time - previous_time).toSec();

    // ROS_INFO("ct:%d , %d ",current_time.sec, current_time.nsec);
    // ROS_INFO("pt:%d , %d ",previous_time.sec, previous_time.nsec);
    // ROS_INFO("dt:%f ", dt);

    double theta = vth * dt;
    linear_velocity_x += linear_acceleration_x * dt;
    linear_velocity_y += linear_acceleration_y * dt;
    ROS_INFO("NEW_vel---X: %f, Y: %f", linear_velocity_x, linear_velocity_y);
    ROS_INFO("-------------------------");

    previous_time = current_time;

    //填充速度信息话题内容并发布
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = linear_velocity_x;
    twist_msg.linear.y = linear_velocity_y;
    twist_msg.angular.z = angular_velocity_z;


    twist_pub.publish(twist_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_integration");
    ros::NodeHandle nh;

    current_time = ros::Time::now();
    previous_time = ros::Time::now();

    ros::Subscriber imu_sub = nh.subscribe("imu/data_raw", 10, imuCallback);
    twist_pub = nh.advertise<geometry_msgs::Twist>("velocity_data", 1);

    // ros::Rate rate(10);
    ros::spin();

    return 0;
}
