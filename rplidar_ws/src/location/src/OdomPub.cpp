#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <location/speed2D.h>

double vx = 0.0;    //初始速度为0
double vy = 0.0;
double vth = 0.0;

double x = 0.0;     // 初始位置为(0,0)
double y = 0.0;
double theta = 0.0; // 初始角度为0

/*
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
    // 读取速度信息
    vx = msg->linear.x;
    vy = msg->linear.y;
    vth = msg->angular.z;
    // ROS_INFO("vx,vy:%f , %f ",vx, vy);
}
*/
void velocityCallback(const location::speed2D::ConstPtr& msg)
{
    vx = msg->linearX;
    vy = msg->linearY;
    vth = msg->angularZ;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle nh;
    setlocale(LC_ALL,"");

    // 创建发布者和TransformBroadcaster对象
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    tf::TransformBroadcaster odom_broadcaster;
    tf2_ros::TransformBroadcaster O2M_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate loop_rate(20); // 控制程序循环频率为20Hz

    // 获取速度信息
    //imu_based
    // ros::Subscriber velocity_sub = nh.subscribe("velocity_data", 10, velocityCallback);
    //RTK_based
    ros::Subscriber sub = nh.subscribe<location::speed2D>("speed2D_topic",10,velocityCallback);
    
    while (ros::ok())
    {
        current_time = ros::Time::now();
        ROS_INFO("---ODOM activated---");
        // 计算时间间隔
        double dt = (current_time - last_time).toSec();


        ROS_INFO("vx:%f,vy:%f", vx , vy);

        // 计算位移
        double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
        double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
        double delta_th = vth * dt;
        
        ROS_INFO("dt:%f ", dt);
        ROS_INFO("dx,dy:%f , %f ",delta_x, delta_y);
        ROS_INFO("dtheta:%f ",delta_th);
        ROS_INFO("---------------------");
        x += delta_x;
        y += delta_y;
        theta += delta_th;

        // ROS_INFO("x:%f ",x);

        // 构建四元数
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        // 发布/odom消息
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        // 设置位置信息
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // 设置速度信息
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.z = vth;
        odom_pub.publish(odom);


        // 发布TF变换信息
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        //发布odom到map的tf变换
        geometry_msgs::TransformStamped O2M_trans;
        O2M_trans.header.stamp = ros::Time::now();
        O2M_trans.header.frame_id = "map";
        O2M_trans.child_frame_id = "odom";
        O2M_trans.transform.translation.x = 0.0;
        O2M_trans.transform.translation.y = 0.0;
        O2M_trans.transform.translation.z = 0.0;
        O2M_trans.transform.rotation.x = 0.0;
        O2M_trans.transform.rotation.y = 0.0;
        O2M_trans.transform.rotation.z = 0.0;
        O2M_trans.transform.rotation.w = 1.0;

        O2M_broadcaster.sendTransform(O2M_trans);


        last_time = current_time;
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ros::spin();
    return 0;
}
