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
#include <queue>
#include "tcp/message_s.h"
#include "tcp/sendMessage.h"
#include <logger/logger.h>



std::mutex tcp_send_mtx, tcp_recv_mtx; // 定义互斥锁
bool enable_send_tcp, ready_send_tcp;  // 条件变量的条件
ros::Publisher tcp_data_pub, tcp_raw_message_pub, tcp_toGroundData_pub;
std::string ground_station_data;
std_msgs::String tcp_recv_string;

tcp::message_s tcp_message;

std::queue<std::string> send_queue; // 发送缓冲队列
std::mutex queue_mutex;             // 保护队列的锁
std::condition_variable queue_cv;   // 队列非空时的通知

void Tcp_GetShipInfo_CallBack(const tcp::ship_info::ConstPtr &msg)
{
    if (msg->tcp_is_connecting == false){
        // 断开Tcp连接
        // ROS_INFO("Handle header P: close server.");
        // Tcp_Server_Close(client_socket);
    }
    if (msg->ship_is_sending == false){
        enable_send_tcp = false;  // 条件变量改变，禁止Tcp发送
    }
    else{
        enable_send_tcp = true;   // 条件变量改变，允许Tcp发送
    }
}

void TxRequestCallback(const std_msgs::String::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(queue_mutex);
    send_queue.push(msg->data); // 入队
    queue_cv.notify_one();      // 通知发送线程：有活干了！
}

/**
 * @brief  服务器发送船体信息至上位机，间隔为0.1s
 * @param  无
 * @retval 无
 */
void Tcp_SendMessage_Thread(int client_socket)
{
    std_msgs::String tcp_toGroundData;
    
    while (ros::ok())
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        // 等待队列不为空
        queue_cv.wait(lock, []{ return !send_queue.empty(); });
        
        // 取出队首数据
        std::string current_msg = send_queue.front(); 
        send_queue.pop();
        
        lock.unlock(); // 尽早解锁
    
        int tcp_send_flag = Tcp_Server_Send(client_socket, current_msg);
        
        if (tcp_send_flag < 0) {
            LOG_ERROR("Send failed, closing socket: {}", client_socket);
            break; 
        }

        tcp_toGroundData.data = current_msg;
        tcp_toGroundData_pub.publish(tcp_toGroundData); 
        LOG_INFO_FILE("tcp_toGroundData:{}\t\n",  current_msg);
    }

    LOG_INFO("close send thread, socket: {}", client_socket);
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
            LOG_ERROR_FILE("Read tcp data failed!");
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
    LOG_WARN("close socket:{}", client_socket);
    Tcp_Server_Close(client_socket);
}

void ServerAccept_Thread()
{
    struct sockaddr_in ClientAddr;
    socklen_t ClientAddr_len = sizeof(ClientAddr);
    char ip_str[20];

    int server_local_sockfd = Tcp_Server_Init(SERVER_PORT);
    LOG_INFO("accepting...");
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
            LOG_WARN("new client connect!:{}", client_socket);
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
    tcp_toGroundData_pub = nh.advertise<std_msgs::String>("/tcp/toGroundData", 1000);
    ros::Subscriber ship_info_sub = nh.subscribe<tcp::ship_info>("/ship/ship_info", 1000, Tcp_GetShipInfo_CallBack);
    ros::Subscriber tcp_request_sub = nh.subscribe<std_msgs::String>("/tcp/tx_request", 1000, TxRequestCallback);

    /*--- End of ROS INIT ----------------------------------------------------------- */

    // 创建线程，传入线程函数
    std::thread ServerAccept(ServerAccept_Thread); /*--- Tcp Server Init ---*/

    ros::spin();

    // 等待线程结束
    ServerAccept.join();

    return 0;
}