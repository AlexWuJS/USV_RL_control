#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>

ros::Publisher points_pub;
tf2_ros::Buffer tfBuffer;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {


    try {
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("map", "wamv/lidar_wamv_link", ros::Time(0));

        sensor_msgs::PointCloud2 transformed_cloud;
        tf2::doTransform(*msg, transformed_cloud, transformStamped);

        transformed_cloud.header.frame_id = "map";
        points_pub.publish(transformed_cloud);
    } catch(tf2::TransformException &e) {
        ROS_WARN("Failed to transform point cloud: %s", e.what());
    }

}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "point_cloud_transform_node");
    ros::NodeHandle nh;

    ros::Subscriber points_sub = nh.subscribe("/wamv/sensors/lidars/lidar_wamv/points", 10, pointCloudCallback);

    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_map", 1);

    
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::spin();

    return 0;

}