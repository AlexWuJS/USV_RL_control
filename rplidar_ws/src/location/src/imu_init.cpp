#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <cmath>

class IMUNode
{
public:
    IMUNode()
    {
        ros::NodeHandle nh;
        imu_sub_ = nh.subscribe("/imu/data_raw", 100, &IMUNode::imuCallback, this);
        imu_init_pub_ = nh.advertise<sensor_msgs::Imu>("/imu_init", 1);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // 计算X和Y方向的线加速度
        double linear_acceleration_x = msg->linear_acceleration.x;
        double linear_acceleration_y = msg->linear_acceleration.y;
        double linear_acceleration_z = 0.0;
        // 统计计算
        x_acc_sum_ += linear_acceleration_x;
        y_acc_sum_ += linear_acceleration_y;
        acc_count_++;

        double ax = linear_acceleration_x;
        double ay = linear_acceleration_y;
        double az = linear_acceleration_z;
        
        // 到达100个消息时进行计算
        if (acc_count_ == 100)
        {
            // 计算平均值
            double x_acc_avg = x_acc_sum_ / 100.0;
            double y_acc_avg = y_acc_sum_ / 100.0;

            // 计算方差
            double x_acc_variance = 0.0;
            double y_acc_variance = 0.0;

            for (const auto& acc : acc_buffer_)
            {
                x_acc_variance += std::pow(acc.x - x_acc_avg, 2);
                y_acc_variance += std::pow(acc.y - y_acc_avg, 2);
            }

            x_acc_variance /= 100.0;
            y_acc_variance /= 100.0;

            ROS_INFO("x_acc_avg:%f", x_acc_avg);
            ROS_INFO("y_acc_avg:%f", y_acc_avg);

            ROS_INFO("x_acc_variance:%f", x_acc_variance);
            ROS_INFO("y_acc_variance:%f", y_acc_variance);

            ROS_INFO("-------------------------");

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
            // ROS_INFO("realRPY-Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);

            // 将欧拉角重新转换为四元数
            tf::Quaternion quata;
            quata.setRPY(roll, pitch, yaw);


            // 构造要发布的消息
            sensor_msgs::Imu init_msg;

            init_msg.linear_acceleration.x = -y_acc_avg;
            init_msg.linear_acceleration.y = x_acc_avg;
            //四元数
            init_msg.orientation.x = quata.x();
            init_msg.orientation.y = quata.y();
            init_msg.orientation.z = quata.z();
            init_msg.orientation.w = quata.w();

            // 发布消息
            imu_init_pub_.publish(init_msg);

            // 重置统计变量
            x_acc_sum_ = 0.0;
            y_acc_sum_ = 0.0;
            acc_count_ = 0;
            acc_buffer_.clear();
        }
        else
        {
            // 存储每个时间步的线加速度
            acc_buffer_.push_back(geometry_msgs::Vector3());
        }
    }

private:
    ros::Subscriber imu_sub_;
    ros::Publisher imu_init_pub_;

    double x_acc_sum_ = 0.0;
    double y_acc_sum_ = 0.0;
    int acc_count_ = 0;
    std::vector<geometry_msgs::Vector3> acc_buffer_;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_node");
    IMUNode imu_node;
    ros::spin();

    return 0;
}
