#include "ros/ros.h"
#include <std_msgs/String.h>
#include "tcp/Tcp.h"
#include <tcp/format_conversion.h>


int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "system_control_node");

    ros::NodeHandle nh;

    tcp_message_pub = nh.advertise<tcp::ship_info>("/tcp/message", 10000);

    return 0;
}