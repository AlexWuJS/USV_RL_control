#include "ros/ros.h"
#include <std_msgs/String.h>
#include "tcp/Tcp.h"
#include <control/Motor.h>
#include "tcp/tcp_connection.h"
#include <tcp/format_conversion.h>
#include "tcp/ship_info.h"
#include <mutex>
#include <condition_variable>
#include <thread>
#include <tf2/utils.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sensor_msgs/NavSatFix.h>
#include <map>
#include "tcp/message_s.h"

std::mutex tcp_send_mtx, tcp_recv_mtx;          // 定义互斥锁
std::condition_variable tcp_send_cv;            // 条件变量
bool ready_send_tcp;                            // 条件变量的条件      
ros::Publisher tcp_message_pub, tcp_ori_message_pub;
ros::Publisher rtk_point_pub;
std_msgs::String tcp_recv_string;
tcp::ship_info ship_information;
tcp::message_s tcp_message;
sensor_msgs::Imu imu;                        // 里程计信息
sensor_msgs::NavSatFix rtk;                  // 里程计信息


void odomCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu = *msg;

    ship_information.heading_angle = tf2::getYaw(imu.orientation);  // 当前船体的航向角
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
void Tcp_PubShipMessage_CallBack(const ros::TimerEvent &event)
{
    // tcp_message_pub.publish(ship_information);
}

/**
  * @brief  从节点中读取一帧串口环境信息
  * @param  无
  * @retval 无
  */
void Tcp_SubUsartMessage_CallBack(const std_msgs::String::ConstPtr& msg)
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
        
/**
  * @brief  查看Tcp的状态信息，根据状态信息进行处理
  * @param  无
  * @retval 无
  */
bool Tcp_Get_Status_CallBack(tcp::tcp_connection::Request &req, tcp::tcp_connection::Response &resp)
{
    std::lock_guard<std::mutex> lock(tcp_send_mtx);     // 上锁，自动释放锁   
    if (req.tcp_is_connected != true)       // 关闭TCP服务器通信
    {
        ROS_INFO("Tcp connection closed: recieve command from client.");
        // Tcp_Server_Close();
        ready_send_tcp = false;             // 条件变量改变，禁止Tcp发送
        tcp_send_cv.notify_all();           // 通知所有线程        
    }

   return true;
}

/**
  * @brief  服务器发送船体信息至上位机，间隔为0.1s
  * @param  无
  * @retval 无
  */
void Tcp_SendMessage_Thread(int client_socket)
{
    char ship_info_sbuf[100];

    while (true)
    {
        std::unique_lock<std::mutex> lock(tcp_send_mtx); // 上锁
        while (!ready_send_tcp) tcp_send_cv.wait(lock);

        sprintf(ship_info_sbuf, "R,%.6f,%.6f,%.3f,%.3f,%.3f\n", ship_information.longitude, ship_information.latitude,\
        ship_information.heading_angle, ship_information.voltage_left_battery, ship_information.voltage_right_battery);
        int tcp_send_flag = Tcp_Server_Send(client_socket, ship_info_sbuf);
        lock.unlock(); // 解锁

        if (tcp_send_flag < 0)      // 发送失败
            break;
        else
        ROS_INFO("Tcp server send ship_info_sbuf: %s", ship_info_sbuf);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));    // 在线程中延时 0.1 秒
    }

    ROS_INFO("close send thread,socket: %d", client_socket);
}


/**
  * @brief  Tcp接收上位机发送的信息。
  * @param  无
  * @retval 无
  */
void Tcp_RecvMessage_Thread(int client_socket)
{
    char tcp_rbuf[100];
    FormatConversionClass FormatConversion_Tcp;
    vector<std::string> sub_strings;
    std::vector<sensor_msgs::NavSatFix> point_list;
    std::vector<std::string> test = {"hello","world","!"};

    while (true)
    {
        std_msgs::String ori_message;
        
        // std::lock_guard<std::mutex> lock(tcp_recv_mtx);     // 上锁，自动释放锁
        memset(tcp_rbuf, '\0', sizeof(tcp_rbuf));           // 接收数据前，清空缓存区
        sub_strings.clear();
        ssize_t bytes_received = Tcp_Server_Read(client_socket, tcp_rbuf);       // 线程安全函数
        if (bytes_received <= 0) {
            break;
        }

        ori_message.data = tcp_rbuf;
        tcp_ori_message_pub.publish(ori_message);

        tcp_recv_string.data = tcp_rbuf;
        /* convert to string */
        FormatConversion_Tcp.SplitString(tcp_recv_string.data, sub_strings);

        tcp_message.str_s = sub_strings;
        tcp_message_pub.publish(tcp_message);//

        for (vector<string>::iterator it = sub_strings.begin(); it != sub_strings.end(); it++) {
            switch (FormatConversion_Tcp.recognize_message(*it))
            {
            case 'S':
                FormatConversion_Tcp.handle_header_S(*it);

                if (FormatConversion_Tcp.tcp_is_connecting() == false)
                {  
                    // 断开Tcp连接
                    ROS_INFO("Handle header P: close server.");
                    Tcp_Server_Close(client_socket);
                }
                if (FormatConversion_Tcp.message_is_recieving() == false)
                {
                    ready_send_tcp = false;         // 条件变量改变，禁止Tcp发送
                    tcp_send_cv.notify_all();       // 通知所有线程
                }
                else
                {
                    ready_send_tcp = true;         // 条件变量改变，允许Tcp发送
                    tcp_send_cv.notify_all();       // 通知所有线程                    
                }

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
    ROS_WARN("close socket:%d", client_socket);
    Tcp_Server_Close(client_socket);
}

void ServerAccept_Thread()
{
    struct sockaddr_in ClientAddr;
    socklen_t ClientAddr_len = sizeof(ClientAddr);
    char ip_str[20];

    int server_local_sockfd = Tcp_Server_Init(SERVER_PORT);
    ROS_INFO("accepting...");
    while (true)
    {
        memset(&ClientAddr, 0, sizeof(ClientAddr));

        /* 等待客户端接收 */
        int client_socket = accept(server_local_sockfd, (struct sockaddr* )&ClientAddr, &ClientAddr_len);
        if (-1 == client_socket)
        {
            perror("server accept failed");
        }
        else
        {
            ROS_WARN("new client connect!:%d", client_socket);
            /* show client information */
            inet_ntop(AF_INET, &ClientAddr.sin_addr, ip_str, sizeof(ip_str));
            cout << "The client port: " << ClientAddr.sin_port << endl;
            cout << "The client IP: " << ip_str << endl;

            std::thread receive_thread(Tcp_RecvMessage_Thread, client_socket);
            std::thread send_thread(Tcp_SendMessage_Thread, client_socket);
            receive_thread.detach();
            send_thread.detach(); 
        }
    }
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "tcp_node");

    ros::NodeHandle nh;

    rtk_point_pub = nh.advertise<sensor_msgs::NavSatFix>("/tcp/rtk_point", 10000);
    tcp_ori_message_pub = nh.advertise<std_msgs::String>("/tcp/ori_message", 10000);

    tcp_message_pub = nh.advertise<tcp::message_s>("/tcp/message", 10000);

    ros::Subscriber tcp_environment_sub = nh.subscribe<std_msgs::String>("/usart_message", 1000, Tcp_SubUsartMessage_CallBack);
    ros::Subscriber odom_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, odomCallback);
    ros::Subscriber rtk_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1000, rtkCallback);   
    ros::ServiceServer service = nh.advertiseService("/tcp_connection", Tcp_Get_Status_CallBack);
    /*--- End of ROS INIT ----------------------------------------------------------- */

    // 创建线程，传入线程函数
    std::thread ServerAccept(ServerAccept_Thread);    /*--- Tcp Server Init ---*/


    ros::Timer timer_tcp_pub = nh.createTimer(ros::Duration(0.1), Tcp_PubShipMessage_CallBack);

    ros::spin();

    // 等待线程结束
    ServerAccept.join();

    return 0;
}