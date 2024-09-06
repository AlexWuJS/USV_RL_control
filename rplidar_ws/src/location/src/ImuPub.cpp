#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// 假设你的IMU数据获取方式在这个回调函数中实现，将获取到的IMU数据填充到imu_msg中
void readIMUData(sensor_msgs::Imu& imu_msg)
{
    // 实现读取IMU数据的逻辑，并将数据填充到imu_msg中
    // 具体需要看与硬件的通信
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.angular_velocity.x = 0.1;  // 设置角速度x分量
    imu_msg.angular_velocity.y = 0.1;  // 设置角速度y分量
    imu_msg.angular_velocity.z = 0;  // 设置角速度z分量
    imu_msg.linear_acceleration.x = 0;  // 设置线加速度x分量
    imu_msg.linear_acceleration.y = 0;  // 设置线加速度y分量
    imu_msg.linear_acceleration.z = 0;  // 设置线加速度z分量
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "imu_publisher");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建IMU消息发布器
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 10);

    // 设置循环的频率
    ros::Rate rate(10);  // 假设以10Hz的频率发布IMU数据

    while (ros::ok())
    {
        // 创建一个sensor_msgs::Imu类型的消息
        sensor_msgs::Imu imu_msg;

        // 读取IMU数据并填充到imu_msg中
        readIMUData(imu_msg);

        // 发布IMU消息
        imu_pub.publish(imu_msg);

        // 处理订阅和回调函数的事件
        ros::spinOnce();

        // 按照指定的频率休眠，控制发布频率
        rate.sleep();
    }

    return 0;
}
