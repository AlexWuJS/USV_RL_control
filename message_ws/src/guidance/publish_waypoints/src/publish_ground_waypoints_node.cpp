#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <vector>
#include <std_msgs/String.h>
#include <robot_localization/navsat_conversions.h>
#include <visualization_msgs/Marker.h>
#include "tcp/message_s.h"


ros::Subscriber goal_sub;
nav_msgs::Path waypoints_msg;
geometry_msgs::PoseStamped goal, goal_oen;
ros::Publisher waypoints_pub, point_marker_pub;
visualization_msgs::Marker marker;
std::string utm_zone;

geometry_msgs::PoseStamped latLongtoUTM(double lati_input, double longi_input)
{
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PoseStamped UTM_point_output;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);   // time stamp is not important here
    UTM_point_output.pose.position.x = utm_x;
    UTM_point_output.pose.position.y = utm_y;
    UTM_point_output.pose.position.z = 0.0;

    return UTM_point_output;
}

geometry_msgs::PoseStamped UTMtoMapPoint(geometry_msgs::PoseStamped& UTM_input)
{
    geometry_msgs::PoseStamped map_point_output;
    bool notDone = true;
    // tf::TransformListener listener; //create transformlistener object called listener
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener listener(tfBuffer);

    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_input.header.stamp = ros::Time::now();
            // listener.waitForTransform("odom_local", "utm", time_now, ros::Duration(3.0));
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("odom_local", "utm", time_now, ros::Duration(3.0));
            
            // listener.transformPoint("odom_local", UTM_input, map_point_output);
            tf2::doTransform(UTM_input, map_point_output, transformStamped);
            notDone = false;
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map_point_output;
}

geometry_msgs::Point visualizePoint(geometry_msgs::PoseStamped map_point)
{
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    geometry_msgs::Point pt;
    pt.x = map_point.pose.position.x;
    pt.y = map_point.pose.position.y;
    pt.z = map_point.pose.position.z;

    return pt;
}

void Tcp_SubMessage_CallBack(const tcp::message_s::ConstPtr& msg)
{
    std::vector<std::string> sub_strings = msg->sub_strings;
    nav_msgs::Path waypoints;
    sensor_msgs::NavSatFix point;
    std::vector<sensor_msgs::NavSatFix> points;

    if (sub_strings[0] == "P")
    {
        // FormatConversion_Tcp.handle_header_P(*it, points);
        sub_strings.erase(sub_strings.begin());      // 去掉'P'帧头
        /* 将所有经纬度转为点集 */
        for (size_t i = 0; i < sub_strings.size(); i++)
        {
            if (i % 2 == 0)
                point.longitude = std::stof(sub_strings[i]);       // 经度
            
            if (i % 2 == 1)
            {
                point.latitude = std::stof(sub_strings[i]);                     // 纬度
                points.push_back(point);
            }
        }

        for (auto& point : points){
            geometry_msgs::PoseStamped utm_point = latLongtoUTM(point.latitude,point.longitude);
            geometry_msgs::PoseStamped map_point = UTMtoMapPoint(utm_point);
            waypoints.poses.push_back(map_point);
            geometry_msgs::Point vis_point = visualizePoint(map_point);
            marker.points.push_back(vis_point);
        }
        waypoints_pub.publish(waypoints);
        point_marker_pub.publish(marker);
    }
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "pub_ground_waypoints");
    ros::NodeHandle nh;

    ros::Subscriber tcp_message_sub = nh.subscribe<tcp::message_s>("/tcp/data", 10, Tcp_SubMessage_CallBack);
    waypoints_pub = nh.advertise<nav_msgs::Path>("/waypoints_path", 10);
    point_marker_pub = nh.advertise<visualization_msgs::Marker>("/point", 10);

    ros::spin();
    return 0;
}
