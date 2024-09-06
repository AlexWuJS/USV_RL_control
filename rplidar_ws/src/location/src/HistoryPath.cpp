#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle nh;

    // 创建tf缓存并设置历史数据缓存大小
    tf2_ros::Buffer tfBuffer(ros::Duration(200.0));
    tf2_ros::TransformListener tfListener(tfBuffer);

    // 创建路径消息发布器
    ros::Publisher pathPublisher = nh.advertise<nav_msgs::Path>("/HistoryPath", 10);

    ros::Rate rate(2);  // 发布频率为2Hz

    while (ros::ok())
    {
        try
        {
            // 查询最近一次的odom变换信息
            geometry_msgs::TransformStamped odomTransform;
            odomTransform = tfBuffer.lookupTransform("odom_local", "base_footprint",
                                                     ros::Time(0), ros::Duration(20.0));

            // 构造路径消息，设置header信息
            nav_msgs::Path path;
            path.header.stamp = odomTransform.header.stamp;
            path.header.frame_id = "odom_local";

            // 创建位姿消息
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = odomTransform.transform.translation.x;
            pose.pose.position.y = odomTransform.transform.translation.y;
            pose.pose.position.z = odomTransform.transform.translation.z;
            pose.pose.orientation = odomTransform.transform.rotation;

            // 将位姿消息添加到路径中
            path.poses.push_back(pose);

            // 发布路径消息
            pathPublisher.publish(path);
            ROS_INFO("----------HistoryPath publishing----------");
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
