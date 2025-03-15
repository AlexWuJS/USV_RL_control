#include <iostream>
using namespace std;
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <string>
#include "control/Motor.h"
#include "tcp/tcp_connection.h"
#include "tcp/ship_info.h"
#include "tcp/keyboard.h"
#include "adjust_pid/point_nav.h"


Point_Nav point_nav;
control::Motor thruster_msg;

void Recv_NavSatFix_CallBack(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if (point_nav.recv_first_waypoint(*msg)){
        point_nav.begin_recv();
    }
    else if (point_nav.recv_last_waypoint(*msg)){
        point_nav.update_recv_waypoints();
    }
    else
        point_nav.add_waypoint(*msg);
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    ros::Subscriber rtk_point_sub = nh.subscribe<sensor_msgs::NavSatFix>("/ship/rtk_point", 10, Recv_NavSatFix_CallBack);
    ros::Publisher boat_auto_control_pub = nh.advertise<control::Motor>("/auto_thruster_pwm", 100);
    // ros::Subscriber boat_status_sub = nh.subscribe<tcp::ship_info>("/ship/ship_info", 1000, Tcp_SubBoatStatus_CallBack);
    // ros::ServiceClient client = nh.serviceClient<tcp::tcp_connection>("/tcp_connection");
    ros::Rate rate(1);
    ROS_INFO("point_nav_node run success.");
    while (ros::ok())
    {
        point_nav.run();
        std::pair<int, int> current_pwm = point_nav.get_thruster_pwm();
        thruster_msg.left_thruster_pwm = current_pwm.first;
        thruster_msg.right_thruster_pwm = current_pwm.second;
        boat_auto_control_pub.publish(thruster_msg);
        ros::spinOnce();

        rate.sleep();// delay 100ms
    }

    return 0;
}
