#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "logger/logger.h" // 包含你自定义的 logger 头文件

// 回调函数
void FixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // 1. 获取经纬度
    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude; // 可选：高度

    // 2. 获取协方差 (3x3 矩阵，展平成 9 个元素的数组)
    // 通常我们最关心对角线元素 (方差): [0]东向, [4]北向, [8]天向 (大概率，取决于坐标系)
    // 这里为了完整，我们打印前3个或者对角线，或者全部
    // 为了日志整洁，这里演示打印对角线（位置不确定度）
    double cov_x = msg->position_covariance[0];
    double cov_y = msg->position_covariance[4];
    double cov_z = msg->position_covariance[8];

    // 3. 使用 LOG_SENSOR_INFO 宏记录
    // 这样数据会自动流向 logs/xxxx-xx-xx/sensor.log
    // 且如果是异步日志，不会阻塞主线程
    LOG_SENSOR_INFO("GPS Fix: Lat={:.10f}, Lon={:.10f}, Alt={:.2f} | Cov(diag): [{:.20f}, {:.20f}, {:.20f}]", 
                    lat, lon, alt, cov_x, cov_y, cov_z);

}

int main(int argc, char **argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "record_data_node");
    ros::NodeHandle nh;

    // 打印启动日志 (走系统日志通道)
    LOG_INFO("Record Data Node Started. Listening to /fix ...");

    // 订阅 /fix 话题
    // 队列长度设为 100，防止高频 GPS 数据丢失
    ros::Subscriber sub = nh.subscribe("/fix", 100, FixCallback);

    // 进入循环
    ros::spin();

    // 【重要】节点退出时的清理工作
    LOG_INFO("Record Data Node Stopping...");
    
    // 强制刷新所有待写入的日志，防止 Ctrl+C 导致最后几条数据丢失
    spdlog::shutdown();

    return 0;
}