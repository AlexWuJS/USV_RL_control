#include "ros/ros.h"
#include <std_msgs/String.h>
#include "tcp/Tcp.h"
#include <control/Motor.h>
#include "tcp/tcp_connection.h"
#include <tcp/format_conversion.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <map>
#include "tcp/message_s.h"
#include "tcp/sendMessage.h"

std::mutex tcp_send_mtx, tcp_recv_mtx; // 定义互斥锁
std::condition_variable tcp_send_cv;   // 条件变量
bool enable_send_tcp, ready_send_tcp;  // 条件变量的条件
ros::Publisher tcp_data_pub, tcp_raw_message_pub, tcp_GroundData_pub;
std::string tcp_sub_string;
std_msgs::String tcp_recv_string;

tcp::message_s tcp_message;

/**
 * @brief  查看Tcp的状态信息，根据状态信息进行处理
 * @param  无
 * @retval 无
 */
bool Tcp_Get_Status_CallBack(tcp::tcp_connection::Request &req, tcp::tcp_connection::Response &resp)
{
    std::lock_guard<std::mutex> lock(tcp_send_mtx); // 上锁，自动释放锁
    if (req.tcp_is_connected != true)               // 关闭TCP服务器通信
    {
        ROS_INFO("Tcp connection closed: recieve command from client.");
        // Tcp_Server_Close();
        // enable_send_tcp = false;             // 条件变量改变，禁止Tcp发送
        tcp_send_cv.notify_all(); // 通知所有线程
    }

    return true;
}

void Tcp_GetShipInfo_CallBack(const tcp::ship_info::ConstPtr &msg)
{
    if (msg->tcp_is_connecting == false)
    {
        // 断开Tcp连接
        // ROS_INFO("Handle header P: close server.");
        // Tcp_Server_Close(client_socket);
    }
    if (msg->ship_is_sending == false)
    {
        enable_send_tcp = false;  // 条件变量改变，禁止Tcp发送
        tcp_send_cv.notify_all(); // 通知所有线程
    }
    else
    {
        enable_send_tcp = true;   // 条件变量改变，允许Tcp发送
        tcp_send_cv.notify_all(); // 通知所有线程
    }
}

/**
 * @brief  Tcp开启服务通信，其他话题通过此函数发送Tcp信息
 * @param  无
 * @retval 无
 */
bool Tcp_Srv_SendMessageCallBack(tcp::sendMessage::Request &req, tcp::sendMessage::Response &resp)
{
    std::unique_lock<std::mutex> lock(tcp_send_mtx); // 上锁
    // while (!enable_send_tcp) tcp_send_cv.wait(lock);
    ready_send_tcp = true;
    tcp_sub_string = req.str;

    lock.unlock(); // 解锁

    return true;
}

/**
 * @brief  服务器发送船体信息至上位机，间隔为0.1s
 * @param  无
 * @retval 无
 */
void Tcp_SendMessage_Thread(int client_socket)
{
    char tcp_sbuf[TCP_MAX_SIZE];

    while (true)
    {
        std::unique_lock<std::mutex> lock(tcp_send_mtx); // 上锁
        // while (!enable_send_tcp) tcp_send_cv.wait(lock);

        if (ready_send_tcp)
        {
            ready_send_tcp = false;

            strcpy(tcp_sbuf, tcp_sub_string.c_str());
            int tcp_send_flag = Tcp_Server_Send(client_socket, tcp_sbuf);

            if (tcp_send_flag < 0) // 发送失败
                break;
            else
                tcp_GroundData_pub.publish(tcp_recv_string); // 发送至上位机
        }
        lock.unlock(); // 解锁

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 在线程中延时 0.1 秒
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
    char tcp_rbuf[TCP_MAX_SIZE];
    FormatConversionClass& FormatConversion_Tcp = FormatConversionClass::get_singleton();
    vector<std::string> data_frames;

    while (true)
    {
        // std::lock_guard<std::mutex> lock(tcp_recv_mtx);     // 上锁，自动释放锁
        memset(tcp_rbuf, '\0', sizeof(tcp_rbuf)); // 接收数据前，清空缓存区
        data_frames.clear();
        ssize_t bytes_received = Tcp_Server_Read(client_socket, tcp_rbuf); // 线程安全函数
        if (bytes_received <= 0)
        { // 读取Tcp数据失败
            break;
        }

        tcp_recv_string.data = tcp_rbuf;
        tcp_raw_message_pub.publish(tcp_recv_string);
        /* convert to string */
        FormatConversion_Tcp.SplitString(tcp_recv_string.data, data_frames);    // 拆解多组数据帧
        for (auto& data_frame: data_frames){
            vector<std::string> tcp_segments = FormatConversion_Tcp.get_value_from_string(data_frame, ',');     // 拆解每组数据帧的数据块
            tcp_message.sub_strings = tcp_segments;
            tcp_data_pub.publish(tcp_message);
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
        int client_socket = accept(server_local_sockfd, (struct sockaddr *)&ClientAddr, &ClientAddr_len);
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

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, " ");
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "tcp_node");

    ros::NodeHandle nh;

    tcp_raw_message_pub = nh.advertise<std_msgs::String>("/tcp/raw_data", 1000);
    tcp_data_pub = nh.advertise<tcp::message_s>("/tcp/data", 1000);
    tcp_GroundData_pub = nh.advertise<std_msgs::String>("/tcp/GroundData", 1000);
    ros::Subscriber ship_info_sub = nh.subscribe<tcp::ship_info>("/ship/ship_info", 1000, Tcp_GetShipInfo_CallBack);

    ros::ServiceServer service_connection = nh.advertiseService("/tcp/connection", Tcp_Get_Status_CallBack);
    ros::ServiceServer service_send_message = nh.advertiseService("/tcp/communication", Tcp_Srv_SendMessageCallBack);
    /*--- End of ROS INIT ----------------------------------------------------------- */

    // 创建线程，传入线程函数
    std::thread ServerAccept(ServerAccept_Thread); /*--- Tcp Server Init ---*/

    ros::spin();

    // 等待线程结束
    ServerAccept.join();

    return 0;
}