#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

double origin_x = 0.0;
double origin_y = 0.0;
double origin_z = 0.0;
double last_x = 0.0;
double last_y = 0.0;
double last_z = 0.0;

ros::Publisher odom_pub, motion_path_pub;
nav_msgs::Odometry odom_local;
ros::CallbackQueue myQueue;
nav_msgs::Path motion_path;
geometry_msgs::PoseStamped pose_msg;
// void callback(const nav_msgs::Odometry::ConstPtr& gps_odom, const sensor_msgs::Imu::ConstPtr& imu_data){
//     odom_local.pose.pose.position = gps_odom->pose.pose.position;
//     odom_local.pose.pose.orientation = imu_data->orientation;

//     odom_pub.publish(odom_local);
// }


void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& gps_odom) {
    // 处理gpsOdom
    // 如果是第一次执行，以当前UTM坐标为odom的原点
    if (origin_x == 0.0 && origin_y == 0.0 && origin_z == 0.0) {
        origin_x = gps_odom->pose.pose.position.x;
        origin_y = gps_odom->pose.pose.position.y;
        origin_z = gps_odom->pose.pose.position.z;

        double dx = 0.0; 
        double dy = 0.0; 
        double dz = 0.0; 
    }
    else{ 
    // 根据UTM坐标更新，计算局部位置变化--
    double dx = gps_odom->pose.pose.position.x - origin_x; // Change in easting
    double dy = gps_odom->pose.pose.position.y - origin_y; // Change in northing
    double dz = gps_odom->pose.pose.position.z - origin_z; // Change in altitude

    // 更新局部里程计--
    odom_local.pose.pose.position.x = dx;
    odom_local.pose.pose.position.y = -dy;
    odom_local.pose.pose.position.z = dz;

    odom_local.header = gps_odom->header;
    }
    
    // 保留本次UTM坐标位置，以便下次循环时计算
    last_x = gps_odom->pose.pose.position.x;
    last_y = gps_odom->pose.pose.position.y;
    last_z = gps_odom->pose.pose.position.z; 

    // 发布无人艇的运行轨迹
    motion_path.header.frame_id = "map";           // 指定消息或数据的参考坐标系，position就是在这个坐标系下的
    motion_path.header.stamp = ros::Time::now();

    pose_msg.header.frame_id = "odom_local"; 
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = odom_local.pose.pose.position.x; 
    pose_msg.pose.position.y = odom_local.pose.pose.position.y; 
    pose_msg.pose.position.z = 0.0; 
    pose_msg.pose.orientation = odom_local.pose.pose.orientation; 
    motion_path.poses.push_back(pose_msg);
    motion_path_pub.publish(motion_path);   

}
void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data) {
    // 处理IMU数据:直接读取四元数
    odom_local.pose.pose.orientation = imu_data->orientation;
}
void combinedCallback() {
    myQueue.callAvailable();
    odom_pub.publish(odom_local);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "odom_processor");
    ros::NodeHandle nh;

    tf2_ros::StaticTransformBroadcaster tf_broadcaster_;
    ros::Subscriber gps_odom_sub = nh.subscribe("/gpsOdom", 10, gpsOdomCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, imuCallback);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_local", 1);
    motion_path_pub = nh.advertise<nav_msgs::Path>("/robot_path", 1);

    // 循环调用自定义回调函数
    ros::Rate loop_rate(10);        // 每秒10次
    while (ros::ok()) {
        combinedCallback();

            // 发布odom_local到utm的静态坐标变换
        // 创建坐标系信息
        geometry_msgs::TransformStamped utm_tf;
        //----设置头信息
        utm_tf.header.seq = 100;
        utm_tf.header.stamp = ros::Time::now();
        utm_tf.header.frame_id = "odom_local";
        //----设置子级坐标系
        utm_tf.child_frame_id = "utm";
        //----设置子级相对于父级的偏移量
        utm_tf.transform.translation.x = origin_x;
        utm_tf.transform.translation.y = origin_y;
        utm_tf.transform.translation.z = origin_z;
        utm_tf.transform.rotation.x = 0.0;
        utm_tf.transform.rotation.y = 0.0;
        utm_tf.transform.rotation.z = 0.0;
        utm_tf.transform.rotation.w = 1.0;
        // 广播器发布坐标系信息
        tf_broadcaster_.sendTransform(utm_tf);   
        
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
