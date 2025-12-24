#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <map>
#include <unistd.h>
#include <arpa/inet.h>
#include "tcp/Tcp.h"

// 静态变量声明
static int connfd_server; // 服务器连接文件描述符
static int sockfd_client; // 客户端套接字文件描述符

int server_local_sockfd = 0; // 服务器本地套接字文件描述符

/**
 * @brief 初始化TCP服务器
 * @param port 服务器端口号
 * @return 成功返回服务器套接字文件描述符，失败返回-1
 */
int Tcp_Server_Init(uint16_t port)
{
    struct sockaddr_in ServerAddr;

    memset(&ServerAddr, 0, sizeof(ServerAddr));

    // 填充服务器地址结构
    ServerAddr.sin_family = AF_INET;                // 使用IPv4
    ServerAddr.sin_addr.s_addr = htonl(INADDR_ANY); // 监听所有可用接口
    ServerAddr.sin_port = htons(port);              // 将端口号转换为网络字节序

    // 创建套接字 // 创建socket
    server_local_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (-1 == server_local_sockfd)
    {
        perror("server create socket failed");
        return -1;
    }

     int opt = 1;
    if (setsockopt(server_local_sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt failed");
        close(server_local_sockfd);
        exit(EXIT_FAILURE);
    }

    // 绑定socket到地址
    if (-1 == bind(server_local_sockfd, (struct sockaddr *)&ServerAddr, sizeof(ServerAddr)))
    {
        perror("server bind address failed");
        return -1;
    }
    cout << "listening...";

    // 监听客户端连接，监听端口
    if (-1 == listen(server_local_sockfd, 50)) // 最大连接数为50，可根据需要修改
    {
        perror("server listen failed");
        return -1;
    }
    cout << "complete" << endl;

    return server_local_sockfd;
}

/**
 * @brief 服务器发送数据 (Char* 版本，支持指定长度)
 * @param client_socket 客户端套接字
 * @param buf 要发送的数据缓冲区
 * @param len 要发送的数据长度 (字节数)
 * @return 成功返回0，失败返回负值
 */
int Tcp_Server_Send(int client_socket, const char *buf, int len)
{
    // 1. 检查套接字有效性
    if (client_socket <= 0)
    {
        cout << "[Tcp Error] Server send failed: Invalid socket." << endl;
        return -1;
    }

    // 2. 检查数据有效性
    if (buf == nullptr || len <= 0)
    {
        cout << "[Tcp Error] Server send failed: Empty data or invalid length." << endl;
        return -1;
    }

    // 3. 安全检查：防止发送超过缓冲区定义的最大长度 (可选，视需求而定)
    if (len > TCP_MAX_SIZE) 
    {
        cout << "[Tcp Warning] Data too long, truncated to TCP_MAX_SIZE." << endl;
        len = TCP_MAX_SIZE; 
    }

    ssize_t sent_bytes = send(client_socket, buf, len, 0);

    if (-1 == sent_bytes)
    {
        perror("server send failed");
        // 为了防止打印乱码，这里只打印前一部分
        printf("Failed to send data (len=%d)\n", len); 
        return -2;
    }

    return 0;
}

/**
 * @brief 服务器发送数据 (String 重载版本)
 * @param client_socket 客户端套接字
 * @param data 要发送的 string 数据
 * @return 成功返回0，失败返回负值
 */
int Tcp_Server_Send(int client_socket, const std::string &data)
{
    // 直接调用 char* 版本，自动获取长度
    return Tcp_Server_Send(client_socket, data.c_str(), static_cast<int>(data.length()));
}

/**
 * @brief 服务器接收数据
 * @param client_socket 客户端套接字
 * @param buf 接收数据的缓冲区
 * @return 成功返回接收到的字节数，失败返回负值
 */
int Tcp_Server_Read(int client_socket, char *buf)
{
    // 检查套接字有效性
    if (client_socket <= 0)
    {
        cout << "Server read failed! Please check your network, maybe your socket is not running as expected." << endl;
        return -1;
    }

    // 接收数据
    int bytes_received = recv(client_socket, buf, TCP_MAX_SIZE, 0);
    if (bytes_received < 0)
    {
        perror("server read failed");
        return -2;
    }
    else if (bytes_received == 0) // 对端关闭了连接
    {
        cout << "Server read failed: Connection closed." << endl;
        return -3;
    }

    return bytes_received;
}

/**
 * @brief 关闭服务器连接
 * @param client_socket 要关闭的客户端套接字
 */
void Tcp_Server_Close(int client_socket)
{
    close(client_socket);
}

/**
 * @brief 初始化TCP客户端
 * @param port 服务器端口号
 * @param IP 服务器IP地址
 * @return 成功返回客户端套接字文件描述符，失败返回-1
 */
int Tcp_Client_Init(uint16_t port, const char *IP)
{
    struct sockaddr_in ServerAddr;
    socklen_t ServerAddr_len = sizeof(ServerAddr);

    memset(&ServerAddr, 0, sizeof(ServerAddr));

    // 填充服务器地址结构
    ServerAddr.sin_family = AF_INET;
    ServerAddr.sin_port = htons(SERVER_PORT);
    
    if (-1 == inet_pton(AF_INET, IP, &ServerAddr.sin_addr.s_addr))
    {
        perror("client get IP failed");
        return -1;
    }

    // 创建套接字
    sockfd_client = socket(AF_INET, SOCK_STREAM, 0);
    if (-1 == sockfd_client)
    {
        perror("create socket failed");
        return -1;
    }

    // 连接服务器
    cout << "connecting..." << endl;
    while (-1 == connect(sockfd_client, (struct sockaddr *)&ServerAddr, ServerAddr_len))
    {
        perror("client connect failed");
    }
    cout << "client: connect success!" << endl;

    return sockfd_client;
}

/**
 * @brief 客户端发送数据
 * @param buf 要发送的数据缓冲区
 * @return 成功返回0，失败返回-1
 */
int Tcp_Client_Send(const char *buf)
{
    static int n = 0;

    // 检查套接字有效性
    if (sockfd_client <= 0)
    {
        cout << "Client send failed! Please check your network, maybe your socket is not running as expected." << endl;
        return -1;
    }

    // 发送数据
    if (-1 == send(sockfd_client, buf, TCP_MAX_SIZE, 0))
    {
        perror("client send failed");
        printf("This message failed to send:\r\n\t%s\r\n", buf);
        return -1;
    }
    else
    {
        cout << "client send message: " << ++n << endl;
    }

    return 0;
}

/**
 * @brief 客户端接收数据
 * @param buf 接收数据的缓冲区
 * @return 成功返回0，失败返回-1
 */
int Tcp_Client_Read(char *buf)
{
    // 检查套接字有效性
    if (sockfd_client <= 0)
    {
        cout << "Client read failed! Please check your network, maybe your socket is not running as expected." << endl;
        return -1;
    }

    // 接收数据
    if (-1 == recv(sockfd_client, buf, TCP_MAX_SIZE, 0))
    {
        perror("client read failed");
        return -1;
    }
    cout << "client read: " << buf << endl;

    return 0;
}

/**
 * @brief 关闭客户端连接
 */
void Tcp_Client_Close(void)
{
    close(sockfd_client);
}
