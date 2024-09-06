#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::Publisher test_pub = nh.advertise<sensor_msgs::NavSatFix>("/test", 1000);
 
    ROS_INFO("hello");
    ros::Rate r(1);
    while (ros::ok())
    {
       sensor_msgs::NavSatFix a;
       a.header.stamp = ros::Time::now();
        test_pub.publish(a);

        r.sleep();
    }

    ros::spin();

    return 0;

}