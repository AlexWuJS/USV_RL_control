#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <map>
#include <unistd.h>
#include <arpa/inet.h>
#include "tcp/Tcp.h"


static int connfd_server;
static int sockfd_client;


int server_local_sockfd = 0;

int Tcp_Server_Init(uint16_t port)
{
    struct sockaddr_in ServerAddr;
    
    memset(&ServerAddr, 0, sizeof(ServerAddr));

    //填充变量
    ServerAddr.sin_family = AF_INET;
    ServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    ServerAddr.sin_port = htons(SERVER_PORT);

    server_local_sockfd = socket(AF_INET, SOCK_STREAM, 0);    /* get socket descriptor */
    if (-1 == server_local_sockfd)
    {
        perror("server create socket failed");
        return -1;
    }
    /* 绑定地址 */
    if (-1 == bind(server_local_sockfd, (struct sockaddr*)&ServerAddr, sizeof(ServerAddr)))
    {
        perror("server bind address failed");
        return -1;
    }
    cout << "listening...";
      /* 监听客户端的端口信息 */
    if (-1 == listen(server_local_sockfd, 50)) // 待修改
    {
        perror("server listen failed");
        return -1;
    }
    cout << "complete" << endl;

    return server_local_sockfd;
}

int Tcp_Server_Send(int client_socket, const char* buf)
{
    /* check socket */
    if (client_socket <= 0)
    {
        cout << "Server send failed! Please check your network, maybe your socket is no run as expected." << endl;
        return -1;
    }
    /* send buf */
    if (-1 == send(client_socket, buf, TCP_MAX_SIZE, 0))
    {
        perror("server send failed");
        printf("This message is failed to send:\r\n\t%s\r\n", buf);
        return -2;
    }

    return 0;
}

// int Tcp_Reconnect(int server_local_sockfd)
// {
//     // 关闭以前的连接
//     Tcp_Server_Close();

//     /* 等待客户端接收 */
//     connfd_server = accept(server_local_sockfd, (struct sockaddr* )&ClientAddr, &ClientAddr_len);
//     if (-1 == connfd_server)
//     {
//         perror("server re_accept failed");
//         return -1;
//     }
    
//     return 0;
// }

int Tcp_Server_Read(int client_socket, char* buf)
{
    /* check socket */
    if (client_socket <= 0)
    {
        cout << "Server read failed! Please check your network, maybe your socket is no run as expected." << endl;
        return -1;
    }
    /* 阻塞等待 */
    int bytes_received = recv(client_socket, buf, TCP_MAX_SIZE, 0);
    if (bytes_received < 0)
    {
        perror("server read failed");
        return -2;
    }
    else if (bytes_received == 0)    // 对端关闭了连接
    {
        cout << "Server read failed: Connection closed." << endl;
        return -3;
    }
    
    return bytes_received;
}

void Tcp_Server_Close(int client_socket)
{
    close(client_socket);
}

/**
* 线程初始化函数（客户端）。
* 阻塞等待通信。建议利用多线程。
**/
int Tcp_Client_Init(uint16_t port, const char* IP)
{
    struct sockaddr_in ServerAddr;    
    socklen_t ServerAddr_len = sizeof(ServerAddr);

    memset(&ServerAddr, 0, sizeof(ServerAddr));
    
    //填充变量
    ServerAddr.sin_family = AF_INET;
    ServerAddr.sin_port = htons(SERVER_PORT);
    if (-1 == inet_pton(AF_INET, IP, &ServerAddr.sin_addr.s_addr))
    {
        perror("client get IP failed");
        return -1;
    }
    /* socket init */
    sockfd_client = socket(AF_INET, SOCK_STREAM, 0);    /* get socket descriptor */
    if (-1 == sockfd_client)
    {
        perror("create socket failed");
        return -1;
    }
    /* connect with server */
    cout << "connecting..." << endl;
    while (-1 == connect(sockfd_client, (struct sockaddr *)&ServerAddr, ServerAddr_len))
    {
        perror("client connect failed");
    }
    cout << "client: connect succeess!" << endl;

    return sockfd_client;
}

int Tcp_Client_Send(const char* buf)
{
    static int n = 0;

    /* check socket */
    if (sockfd_client <= 0)
    {
        cout << "Client send failed! Please check your network, maybe your socket is no run as expected." << endl;
        return -1;
    }

    if (-1 == send(sockfd_client, buf, TCP_MAX_SIZE, 0))
    {
        perror("client send failed");
        printf("This message is failed to send:\r\n\t%s\r\n", buf);
        return -1;
    }
    else
    {
        cout << "client send message: " << ++n << endl;  
    } 

    return 0;
}

int Tcp_Client_Read(char* buf)
{
    /* check socket */
    if (sockfd_client <= 0)
    {
        cout << "Client read failed! Please check your network, maybe your socket is no run as expected." << endl;
        return -1;
    }
        /* 阻塞等待 */
    if (-1 == recv(sockfd_client, buf, TCP_MAX_SIZE, 0))
    {
        perror("client read failed");
        return -1;
    }
    cout << "client read: " << buf << endl;

    return 0;
}

void Tcp_Client_Close(void)
{
    close(sockfd_client);
}
