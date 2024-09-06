#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

double origin_x = 0.0;
double origin_y = 0.0;
double origin_z = 0.0;
double last_x = 0.0;
double last_y = 0.0;
double last_z = 0.0;

ros::Publisher odom_local_pub;
nav_msgs::Odometry odom_local;
geometry_msgs::Quaternion odom_quat;

tf::TransformBroadcaster odom_broadcaster;
tf2_ros::TransformBroadcaster O2M_broadcaster;

void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 如果是第一次执行，以当前UTM坐标为odom的原点
    if (origin_x == 0.0 && origin_y == 0.0 && origin_z == 0.0) {
        origin_x = msg->pose.pose.position.x;
        origin_y = msg->pose.pose.position.y;
        origin_z = msg->pose.pose.position.z;

        double dx = 0.0; 
        double dy = 0.0; 
        double dz = 0.0; 
    }
    else{
    // 根据UTM坐标更新，计算局部位置变化
    double dx = msg->pose.pose.position.x - last_x; // Change in easting
    double dy = msg->pose.pose.position.y - last_y; // Change in northing
    double dz = msg->pose.pose.position.z - last_z; // Change in altitude

    // 更新局部里程计
    odom_local.pose.pose.position.x += dx;
    odom_local.pose.pose.position.y += dy;
    odom_local.pose.pose.position.z += dz;

    odom_local.header = msg->header;

    // 姿态信息照搬
    odom_local.pose.pose.orientation = msg->pose.pose.orientation;
    
    }
    
    // 设置速度信息
    odom_local.twist.twist.linear.x = 0.0;
    odom_local.twist.twist.linear.y = 0.0;
    odom_local.twist.twist.angular.z = 0.0;

    // 发布odom_local话题
    odom_local_pub.publish(odom_local);
    
    // 发布TF变换信息
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom_local";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = odom_local.pose.pose.position.x;
    odom_trans.transform.translation.y = odom_local.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    //发布odom到map的tf变换
    geometry_msgs::TransformStamped O2M_trans;
    O2M_trans.header.stamp = ros::Time::now();
    O2M_trans.header.frame_id = "map";
    O2M_trans.child_frame_id = "odom_local";
    O2M_trans.transform.translation.x = 0.0;
    O2M_trans.transform.translation.y = 0.0;
    O2M_trans.transform.translation.z = 0.0;
    O2M_trans.transform.rotation.x = 0.0;
    O2M_trans.transform.rotation.y = 0.0;
    O2M_trans.transform.rotation.z = 0.0;
    O2M_trans.transform.rotation.w = 1.0;

    O2M_broadcaster.sendTransform(O2M_trans);

    // 保留本次UTM坐标位置，以便下次循环时计算
    last_x = msg->pose.pose.position.x;
    last_y = msg->pose.pose.position.y;
    last_z = msg->pose.pose.position.z;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_odometry_node");

    ros::NodeHandle nh;

    // 创建gpsOdom订阅器
    ros::Subscriber gps_odom_sub = nh.subscribe("/gpsOdom", 10, gpsOdomCallback);

    // 创建odom_local发布器
    odom_local_pub = nh.advertise<nav_msgs::Odometry>("/odom_local", 10);


    ros::spin();

    return 0;
}
