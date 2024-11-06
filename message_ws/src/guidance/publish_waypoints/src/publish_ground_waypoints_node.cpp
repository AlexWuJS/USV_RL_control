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
#include <robot_localization/navsat_conversions.h>


ros::Subscriber goal_sub;
nav_msgs::Path waypoints_msg;
geometry_msgs::PoseStamped goal, goal_oen;
ros::Publisher waypoints_pub;
tf2_ros::Buffer tfBuffer;

geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PointStamped UTM_point_output;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = utm_x;
    UTM_point_output.point.y = utm_y;
    UTM_point_output.point.z = 0;

    return UTM_point_output;
}

geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
            listener.transformPoint("odom", UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map_point_output;
}

void Tcp_SubMessage_CallBack(const tcp::message_s::ConstPtr& msg)
{
    vector<std::string> sub_strings = msg->sub_strings;

    if (sub_strings[0] == "P")
    {
        point_list.clear();
        FormatConversion_Tcp.handle_header_P(*it, point_list);

        for (auto& point : points){
            geometry_msgs::PointStamped utm_point = latLongtoUTM(point.,point.);
            geometry_msgs::PointStamped map_point = UTMtoMapPoint(utm_point);
            waypoints.pushback(map_point);
        }
        waypoints_pub.publish(waypoints);
    }
}

move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point)
{
    move_base_msgs::MoveBaseGoal goal;

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
    goal.target_pose.pose.position.y = map_point.point.y; //specify y goal
    goal.target_pose.pose.orientation.w = 1.0;    // don't care about heading because we aren't actually acheiving our goal

    return goal;
}

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

    ros::Subscriber tcp_message_sub = nh.subscribe<tcp::message_s>("/tcp/data", 10000, Tcp_SubMessage_CallBack);
    waypoints_pub = nh.advertise<nav_msgs::Path>("/waypoints_path", 10);

    tf2_ros::TransformListener tfListener(tfBuffer);    // tf的转换监听器

    ros::spin();

    return 0;

}