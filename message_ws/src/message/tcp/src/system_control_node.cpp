#include "ros/ros.h"
#include <std_msgs/String.h>
#include "tcp/Tcp.h"
#include <tcp/format_conversion.h>
#include <tf2/utils.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "tcp/ship_info.h"
#include "tcp/sendMessage.h"
#include <thread>

tcp::ship_info ship_information;
sensor_msgs::Imu imu;                        // 里程计信息
sensor_msgs::NavSatFix rtk;                  // 里程计信息
ros::Publisher ship_info_pub, rtk_point_pub;
ros::ServiceClient client;


void odomCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu = *msg;

    ship_information.heading_angle = tf2::getYaw(imu.orientation);  // 当前船体的航向角
}


void rtkCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    rtk = *msg;

    ship_information.latitude = rtk.latitude;       // 当前船体经度
    ship_information.longitude = rtk.longitude;     // 当前船体纬度
    
}

void TcpSend_Thread()
{
    tcp::sendMessage tcp_send_message;
    char ship_info_sbuf[100];

    while (true)
    {
        if (ship_information.ship_is_sending)
        {
            sprintf(ship_info_sbuf, "R,%.6f,%.6f,%.3f,%.3f,%.3f\n", ship_information.longitude, ship_information.latitude,\
                ship_information.heading_angle, ship_information.voltage_left_battery, ship_information.voltage_right_battery);

            tcp_send_message.request.str = ship_info_sbuf;

            client.call(tcp_send_message);
        }            
    }
}

/**
  * @brief  Tcp发布处理完的船体信息。与接收分开，Tcp没有通信时也发布。（串口的会更新）
  * @param  无
  * @retval 无
  */
void PubShipMessage_CallBack(const ros::TimerEvent &event)
{
    ship_info_pub.publish(ship_information);
}

/**
  * @brief  从节点中读取一帧串口环境信息
  * @param  无
  * @retval 无
  */
void SubUsartMessage_CallBack(const std_msgs::String::ConstPtr& msg)
{
    FormatConversionClass FormatConversion_usart;
    vector<std::string> sub_strings;

    /* convert to string */
    int flag = FormatConversion_usart.SplitString(msg->data, sub_strings);
    if (flag == -1){
        cout << "Tcp_SubUsart split string " << msg->data << ": find headers error! String does not fit for transmit protocol." << endl;
    }

    for (vector<string>::iterator it = sub_strings.begin(); it != sub_strings.end(); it++) {
        switch (FormatConversion_usart.recognize_message(*it))
        {
            case 'M':
                FormatConversion_usart.handle_header_M(*it);
                ship_information.voltage_left_battery = FormatConversion_usart.get_voltage_left_battery();
                ship_information.voltage_right_battery = FormatConversion_usart.get_voltage_right_battery();
            break;

            default:
                ROS_INFO("Sub_Usart error: Please check the message, maybe the message is not fit for the protocol!");
            break;
        }
    }
}

void SubMessage_CallBack(const std_msgs::String::ConstPtr& msg)
{
    vector<std::string> sub_strings;
    FormatConversionClass FormatConversion_Tcp;
    std::vector<sensor_msgs::NavSatFix> point_list;

    sub_strings.push_back(msg->data);

    for (vector<string>::iterator it = sub_strings.begin(); it != sub_strings.end(); it++) {
        switch (FormatConversion_Tcp.recognize_message(*it))
        {
            case 'S':
                FormatConversion_Tcp.handle_header_S(*it);

                ship_information.push_is_open = FormatConversion_Tcp.push_is_open();
                ship_information.ship_is_sending = FormatConversion_Tcp.message_is_recieving();
                ship_information.tcp_is_connecting = FormatConversion_Tcp.tcp_is_connecting();
                ship_information.control_mode = FormatConversion_Tcp.control_mode();
                ship_information.boat_keep_nav = FormatConversion_Tcp.boat_keep_nav();
            break;

            case 'P':
                point_list.clear();
                FormatConversion_Tcp.handle_header_P(*it, point_list);

                for (vector<sensor_msgs::NavSatFix>::iterator point_ptr = point_list.begin(); point_ptr != point_list.end(); point_ptr++)
                {
                    int i;
                    rtk_point_pub.publish(*point_ptr);
                    ROS_WARN("pub %d point.",++i);
                }

            break;

            case 'K':
                FormatConversion_Tcp.handle_header_K(*it);

                ship_information.keyboard.W = FormatConversion_Tcp.keyboard().W;
                ship_information.keyboard.A = FormatConversion_Tcp.keyboard().A;
                ship_information.keyboard.S = FormatConversion_Tcp.keyboard().S;
                ship_information.keyboard.D = FormatConversion_Tcp.keyboard().D;
            break;
            
            case 'T':
            break;

            case 'X':
                FormatConversion_Tcp.handle_header_X(*it, ship_information);
            break;

            default:
                ROS_INFO("Message recognize error: \"%s\" is not fit for the protocol!", (*it).c_str());
            break;
        }
    }
}


int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "system_control_node");

    ros::NodeHandle nh;

    ship_info_pub = nh.advertise<tcp::ship_info>("/ship/ship_info", 10000);
    rtk_point_pub = nh.advertise<sensor_msgs::NavSatFix>("/ship/rtk_point", 10000);

    ros::Subscriber odom_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, odomCallback);
    ros::Subscriber rtk_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1000, rtkCallback); 
    ros::Subscriber tcp_environment_sub = nh.subscribe<std_msgs::String>("/usart_message", 1000, SubUsartMessage_CallBack);
    ros::Subscriber tcp_message_sub = nh.subscribe<std_msgs::String>("/tcp/raw_data", 10000, SubMessage_CallBack);

    client = nh.serviceClient<tcp::sendMessage>("/tcp/communication");
    ros::service::waitForService("/tcp/communication", -1);

    ros::Timer timer_tcp_pub = nh.createTimer(ros::Duration(0.1), PubShipMessage_CallBack);
    // ros::Timer timer_tcp_send = nh.createTimer(ros::Duration(1), SendTcpMessage_CallBack);

    // 创建线程，传入线程函数
    std::thread TcpSend(TcpSend_Thread);    

    ros::spin();

    // 等待线程结束
    TcpSend.join();

    return 0;
}