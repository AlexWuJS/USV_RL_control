#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <gps_common/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <std_msgs/String.h>


ros::Subscriber goal_sub;
nav_msgs::Path waypoints_msg;
geometry_msgs::PoseStamped goal, goal_oen;
ros::Publisher waypoints_pub;
tf2_ros::Buffer tfBuffer;

void rtk_point_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    double northing, easting;
    std::string zone;

    // 转换为UTM坐标(map)
    gps_common::LLtoUTM(msg->latitude, msg->longitude, northing, easting, zone);

    goal.header.stamp = msg->header.stamp;
    // goal.header.frame_id = msg->header.fram_id;
    // goal.header.stamp = ros::Time::now(); 
    goal.header.frame_id = "map";

    goal.pose.position.x = easting;
    goal.pose.position.y = northing;
    goal.pose.position.z = 0.0;

    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;
    // 转换为OEN坐标(odom_local)
    try
    {
        goal_oen = tfBuffer.transform(goal, "odom_local");
    }
    catch(const std::exception& e)
    { 
        std::cerr << e.what() << '\n';
        ROS_WARN("map_tf error.....");
    }

    // waypoints_msg.poses.clear();        // 覆盖的方法来发布路径，所以需要清除点
    waypoints_msg.header.frame_id = "odom_local";
    waypoints_msg.header.stamp = ros::Time::now(); 

    waypoints_msg.poses.push_back(goal_oen);
    waypoints_pub.publish(waypoints_msg);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "pub_path_node");
    ros::NodeHandle nh;

    goal_sub = nh.subscribe("/ship/rtk_point", 1, rtk_point_callback);
    waypoints_pub = nh.advertise<nav_msgs::Path>("/waypoints_path", 10);

    tf2_ros::TransformListener tfListener(tfBuffer);    // tf的转换监听器

    ros::spin();

    return 0;

}