#include "ros/ros.h"
#include <std_msgs/String.h>
#include <logger/logger.h>
#include <iostream>
#include <string>

/**
 * @brief TCP 发送测试节点
 * 该节点用于测试向 /tcp/tx_request 话题发送消息，验证 TCP 通信功能
 */
int main(int argc, char *argv[])
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "tcp_example_node");
    ros::NodeHandle nh;

    // 创建发布者，向 /tcp/tx_request 话题发布消息
    ros::Publisher tcp_tx_pub = nh.advertise<std_msgs::String>("/tcp/tx_request", 1000);

    // 等待发布者连接（确保有订阅者）
    ros::Duration(1.0).sleep();
    
    // 检查是否有订阅者
    if (tcp_tx_pub.getNumSubscribers() == 0)
    {
        LOG_WARN("No subscribers to /tcp/tx_request topic. Make sure tcp_node is running.");
    }
    else
    {
        LOG_INFO("Found %d subscriber(s) to /tcp/tx_request topic", tcp_tx_pub.getNumSubscribers());
    }

    // 设置发布频率（Hz）
    ros::Rate loop_rate(1); // 每秒发送一次

    int message_count = 0;
    std_msgs::String msg;

    LOG_INFO("TCP Example Node started. Sending test messages to /tcp/tx_request...");
    LOG_INFO("Press Ctrl+C to stop.");

    while (ros::ok())
    {
        // 构造测试消息
        message_count++;
        msg.data = "Test message #" + std::to_string(message_count) + " from example_node";
        
        // 发布消息
        tcp_tx_pub.publish(msg);
        
        // 输出日志
        LOG_INFO("Published message: %s", msg.data.c_str());
        
        // 处理回调
        ros::spinOnce();
        
        // 按照设定的频率休眠
        loop_rate.sleep();
    }

    LOG_INFO("TCP Example Node stopped.");
    return 0;
}
