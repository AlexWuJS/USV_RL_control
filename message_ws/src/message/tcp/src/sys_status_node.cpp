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

#include "logger/logger.h"
#include <audio/audio.h>
#include "tcp/network_monitor.h" 


tcp::ship_info ship_information;
sensor_msgs::Imu imu;                        // 里程计信息
sensor_msgs::NavSatFix rtk;                  // 里程计信息
ros::Publisher ship_info_pub, rtk_point_pub, tcp_tx_pub;

// 定义并初始化常量
const sensor_msgs::NavSatFix virtual_start_waypoint = [] {
    sensor_msgs::NavSatFix pos;
    pos.latitude = 91.0;    // 默认纬度
    pos.longitude = 181.0;   // 默认经度
    pos.altitude = 0.0;    // 默认高度
    pos.header.frame_id = "map";  // 可选：设置坐标系
    return pos;
}();

const sensor_msgs::NavSatFix virtual_end_waypoint = [] {
    sensor_msgs::NavSatFix pos;
    pos.latitude = -91.0;    // 默认纬度
    pos.longitude = -181.0;   // 默认经度
    pos.altitude = 0.0;    // 默认高度
    pos.header.frame_id = "map";  // 可选：设置坐标系
    return pos;
}();

void odomCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu = *msg;

    ship_information.heading_angle = tf2::getYaw(imu.orientation)*180/M_PI;  // 当前船体的航向角
}


void rtkCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    rtk = *msg;

    ship_information.latitude = rtk.latitude;       // 当前船体经度
    ship_information.longitude = rtk.longitude;     // 当前船体纬度
}

/**
  * @brief  Tcp发布处理完的船体信息。与接收分开，Tcp没有通信时也发布。（串口的会更新）
  * @param  无
  * @retval 无
  */
void PubShipMessage_CallBack(const ros::TimerEvent &event)
{
    char ship_info_sbuf[100];
    std_msgs::String tcp_sbuf;
    
    ship_info_pub.publish(ship_information);

    // 将状态传输给tcp
    if (ship_information.ship_is_sending)
    {
        sprintf(ship_info_sbuf, "R,%.6f,%.6f,%.3f,%.3f,%.3f\n", ship_information.longitude, ship_information.latitude,\
            ship_information.heading_angle, ship_information.voltage_left_battery, ship_information.voltage_right_battery);
        
        tcp_sbuf.data = ship_info_sbuf;
        tcp_tx_pub.publish(tcp_sbuf);
    }
}

/**
  * @brief  从节点中读取一帧串口环境信息
  * @param  无
  * @retval 无
  */
void SubUsartMessage_CallBack(const std_msgs::String::ConstPtr& msg)
{
    FormatConversionClass& FormatConversion_usart = FormatConversionClass::get_singleton();
    vector<std::string> raw_recv_strings;

    /* convert to string */
    int flag = FormatConversion_usart.SplitString(msg->data, raw_recv_strings);
    if (flag == -1){
        cout << "Tcp_SubUsart split string " << msg->data << ": find headers error! String does not fit for transmit protocol." << endl;
    }

    for (std::string& recv_strings : raw_recv_strings){
        vector<string> sub_strings = FormatConversion_usart.get_value_from_string(recv_strings, ',');
        if (sub_strings[0] == "M" && sub_strings.size() == 8)      // M帧一共有8个数据
        {
            ship_information.voltage_left_battery = stof(sub_strings[1]);
            ship_information.voltage_right_battery = stof(sub_strings[2]);
            ship_information.boat_is_active = stof(sub_strings[3]);
            ship_information.boat_is_running = stof(sub_strings[4]);
            ship_information.boat_is_smart_remote_mode = stof(sub_strings[5]);
        }
        else
            // ROS_INFO("Sub_Usart error: Please check the message, maybe the message is not fit for the protocol!");
        ;
    }
}

void SubMessage_CallBack(const std_msgs::String::ConstPtr& msg)
{
    vector<std::string> sub_strings;
    FormatConversionClass& FormatConversion_Tcp = FormatConversionClass::get_singleton();
    std::vector<sensor_msgs::NavSatFix> point_list;

    sub_strings.push_back(msg->data);

    for (vector<string>::iterator it = sub_strings.begin(); it != sub_strings.end(); it++) {
        switch (FormatConversion_Tcp.recognize_message(*it))
        {
            case 'S':
                FormatConversion_Tcp.handle_header_S(*it);

                ship_information.push_is_open = FormatConversion_Tcp.push_is_open;
                ship_information.ship_is_sending = FormatConversion_Tcp.is_recieving;
                ship_information.tcp_is_connecting = FormatConversion_Tcp.is_connecting;
                // ship_information.control_mode = FormatConversion_Tcp.control_mode;
                ship_information.is_auto_control = FormatConversion_Tcp.is_auto_control;
                ship_information.is_keyboard_control = FormatConversion_Tcp.is_keyboard_control;
                ship_information.boat_keep_nav = FormatConversion_Tcp.boat_keep_nav;
            break;

            case 'P':
                point_list.clear();

                rtk_point_pub.publish(virtual_start_waypoint);   // 发送一个虚拟航点，用于清空原有航点
                FormatConversion_Tcp.handle_header_P(*it, point_list);
                for (auto& point : point_list)
                {
                    rtk_point_pub.publish(point);
                }
                rtk_point_pub.publish(virtual_end_waypoint);   // 发送一个虚拟航点，用于清空原有航点
            break;

            case 'K':
                FormatConversion_Tcp.handle_header_K(*it);

                ship_information.keyboard.W = FormatConversion_Tcp.keyboard.W;
                ship_information.keyboard.A = FormatConversion_Tcp.keyboard.A;
                ship_information.keyboard.S = FormatConversion_Tcp.keyboard.S;
                ship_information.keyboard.D = FormatConversion_Tcp.keyboard.D;
            break;

            default:
                // ROS_INFO("Message recognize error: \"%s\" is not fit for the protocol!", (*it).c_str());
            break;
        }
    }
}


int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "sys_status_node");

    ros::NodeHandle nh;

    ship_info_pub = nh.advertise<tcp::ship_info>("/ship/ship_info", 1000);
    rtk_point_pub = nh.advertise<sensor_msgs::NavSatFix>("/ship/rtk_point", 10);
    tcp_tx_pub = nh.advertise<std_msgs::String>("/tcp/tx_request", 1000);

    ros::Subscriber odom_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, odomCallback);
    ros::Subscriber rtk_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1000, rtkCallback); 
    ros::Subscriber tcp_environment_sub = nh.subscribe<std_msgs::String>("/usart/data", 1000, SubUsartMessage_CallBack);
    ros::Subscriber tcp_message_sub = nh.subscribe<std_msgs::String>("/tcp/raw_data", 1000, SubMessage_CallBack);

    ros::Timer timer_tcp_pub = nh.createTimer(ros::Duration(0.1), PubShipMessage_CallBack);

    // 获取音频音量参数，默认 -1 (不修改系统音量)
    ros::NodeHandle private_nh("~");
    int audio_volume = -1;
    private_nh.param("audio_volume", audio_volume);

    AudioManager audio_mgr(nh, audio_volume);
    NetworkMonitor net_monitor(audio_mgr);

    // 3. 创建 Timer，绑定类的成员函数
    // 这里的逻辑是：每隔 NET_CHECK_INTERVAL (3秒)，ROS 会自动调用 net_monitor.update()
    ros::Timer timer_net_check = nh.createTimer(
        ros::Duration(NET_CHECK_INTERVAL), 
        &NetworkMonitor::update, 
        &net_monitor
    );
    
    ros::spin();

    return 0;
}