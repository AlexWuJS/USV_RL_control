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
#include "adjust_pid/point_nav.h"
#include <sensor_msgs/Imu.h>
#include <tf2/utils.h>
#include <logger/logger.h>

Point_Nav point_nav;
control::Motor thruster_msg;
sensor_msgs::NavSatFix current_position;
double current_heading_angle;

void rtkCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_position = *msg;
}

void odomCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    current_heading_angle = tf2::getYaw(msg->orientation)*180/M_PI;  // 当前船体的航向角
}

void Recv_NavSatFix_CallBack(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if (point_nav.recv_first_waypoint(*msg)){
        point_nav.begin_recv();
    }
    else if (point_nav.recv_last_waypoint(*msg)){
        point_nav.update_recv_waypoints();
        ROS_INFO("recieve %d waypoints.", point_nav.point_list.size());
        for (auto& point : point_nav.point_list){
            cout << point.latitude << "\t"<<point.longitude<<endl;
        }
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

    ros::Publisher boat_auto_control_pub = nh.advertise<control::Motor>("/auto_thruster_pwm", 100);
    ros::Subscriber rtk_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1000, rtkCallback); 
    ros::Subscriber odom_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, odomCallback);
    ros::Subscriber rtk_point_sub = nh.subscribe<sensor_msgs::NavSatFix>("/ship/rtk_point", 10, Recv_NavSatFix_CallBack);
    // ros::Subscriber boat_status_sub = nh.subscribe<tcp::ship_info>("/ship/ship_info", 1000, Tcp_SubBoatStatus_CallBack);
    // ros::ServiceClient client = nh.serviceClient<tcp::tcp_connection>("/tcp_connection");
    ros::Rate rate(0.1);
    
    // 设置PID参数
    point_nav.setPidParams(nh);
    
    LOG_INFO("point_nav_node run success.");
    LOG_SENSOR_INFO("test sensor info.");
    while (ros::ok())
    {
        point_nav.run(current_position, current_heading_angle);
        std::pair<int, int> current_pwm = point_nav.get_thruster_pwm();
        thruster_msg.left_thruster_pwm = current_pwm.first;
        thruster_msg.right_thruster_pwm = current_pwm.second;
        boat_auto_control_pub.publish(thruster_msg);
        ros::spinOnce();

        rate.sleep();// delay 100ms
    }

    return 0;
}
