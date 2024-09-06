#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 在回调函数中处理接收到的消息数据
    // 这里只是简单地打印出接收到的Odometry消息数据
    ROS_INFO("里程计信息:");
    ROS_INFO("位置: [%f, %f, %f]", msg->pose.pose.position.x, 
                                        msg->pose.pose.position.y,
                                        msg->pose.pose.position.z);
    ROS_INFO("姿态/四元数: [%f, %f, %f, %f]", msg->pose.pose.orientation.x, 
                                              msg->pose.pose.orientation.y,
                                              msg->pose.pose.orientation.z,
                                              msg->pose.pose.orientation.w);

    // 打印线速度信息
    ROS_INFO("线速度: [%f, %f, %f]", 
        msg->twist.twist.linear.x, 
        msg->twist.twist.linear.y, 
        msg->twist.twist.linear.z);
    
    // 打印角速度信息
    ROS_INFO("角速度: [%f, %f, %f]", 
        msg->twist.twist.angular.x, 
        msg->twist.twist.angular.y, 
        msg->twist.twist.angular.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_subscriber");
    ros::NodeHandle nh;
    setlocale(LC_ALL,"");
    // 创建一个订阅者对象，订阅名为"/odom"的话题，消息类型为Odometry
    ros::Subscriber sub = nh.subscribe("/odom", 10, odomCallback);

    // 循环等待接收消息
    ros::spin();

    return 0;
}
