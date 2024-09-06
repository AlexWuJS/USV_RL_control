#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <vector>
#include <mutex>
#include <queue>
#include "tcp/Tcp.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

std::vector<int> client_sockets;
std::mutex client_sockets_mutex;
std::queue<std::pair<int, std::string>> message_queue;
std::mutex message_queue_mutex;

void handle_receive(int client_socket) {
    char buffer[1024];
    ssize_t bytes_received;

    while (true) {
        memset(buffer, '\0', sizeof(buffer));           // 接收数据前，清空缓存区
        ROS_INFO("buffer:%s", buffer);
        bytes_received = Tcp_Server_Read(client_socket, buffer);       // 线程安全函数
        if (bytes_received <= 0) {
            break;
        }

        ROS_INFO("Received message from client: %s", buffer);

        // 将接收到的消息添加到消息队列中
        // message_queue_mutex.lock();
        // message_queue.push(std::make_pair(client_socket, std::string(buffer, bytes_received)));
        // message_queue_mutex.unlock();
    }
    ROS_WARN("close socket:%d", client_socket);
    Tcp_Server_Close(client_socket);

    // 从客户端套接字列表中删除当前客户端套接字
    // client_sockets_mutex.lock();
    // auto it = std::find(client_sockets.begin(), client_sockets.end(), client_socket);
    // if (it != client_sockets.end()) {
    //     client_sockets.erase(it);
    // }
    // client_sockets_mutex.unlock();

    // ROS_INFO("Client disconnected");
}

void handle_send(int client_socket) {
    char test[100];
    while (true) {
        message_queue_mutex.lock();

    memset(test, '\0', sizeof(test));        
    sprintf(test, "hello:%d\n", client_socket);
    int flag = Tcp_Server_Send(client_socket, test);
        if ((flag) < 0)
        {
                    message_queue_mutex.unlock();
            break;

        }

        message_queue_mutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    cout <<"close send:" << client_socket << endl;    
}

void createClientConnection_CallBack()
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
            cout << "new client connect!" << endl;
            /* show client information */
            inet_ntop(AF_INET, &ClientAddr.sin_addr, ip_str, sizeof(ip_str));
            cout << "The client port: " << ClientAddr.sin_port << endl;
            cout << "The client IP: " << ip_str << endl;

            std::thread receive_thread(handle_receive, client_socket);
            std::thread send_thread(handle_send, client_socket);
            receive_thread.detach();
            send_thread.detach(); 
        }
    }
}


int main(int argc, char* argv[])
{
    char client_rbuf[100];

    setlocale(LC_ALL, " ");     
    /*--- ROS INIT ----------------------------------------------------------- */
    ros::init(argc, argv, "tcp_node");

    ros::NodeHandle nh;

    // ros::Subscriber tcp_environment_sub = nh.subscribe<std_msgs::String>("usart_environment_message", 1000, Tcp_SubEnvironmentMessage_CallBack);
    // ros::Publisher tcp_pointer_sub = nh.advertise<std_msgs::String>("message_point_listener", 1000);

    std::thread createClientConnection(createClientConnection_CallBack);

    ros::spin();

    return 0;
}