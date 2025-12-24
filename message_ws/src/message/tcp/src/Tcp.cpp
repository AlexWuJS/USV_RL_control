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

TcpClient::TcpClient() : sock_fd_(-1), is_connected_(false) {
}

TcpClient::~TcpClient() {
    closeConnection();
}

void TcpClient::setSocketTimeout(int fd, int sec) {
    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(struct timeval));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval));
}

bool TcpClient::connectToServer(const string& ip, uint16_t port, int timeout_sec) {
    // 如果已经连着，先断开，防止逻辑混乱
    if (is_connected_) {
        closeConnection();
    }

    // 1. 创建 Socket
    sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) {
        perror("[TcpClient] Create socket failed");
        return false;
    }

    // 2. 设置超时 (防止卡死 ROS)
    setSocketTimeout(sock_fd_, timeout_sec);

    // 3. 准备地址
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) <= 0) {
        perror("[TcpClient] Invalid IP");
        closeConnection();
        return false;
    }

    // 4. 发起连接
    cout << "[TcpClient] Connecting to " << ip << ":" << port << " ..." << endl;
    if (connect(sock_fd_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("[TcpClient] Connect failed");
        closeConnection(); // 失败要清理 fd
        return false;
    }

    is_connected_ = true;
    return true;
}

int TcpClient::sendMsg(const string& data) {
    return sendMsg(data.c_str(), data.length());
}

int TcpClient::sendMsg(const char* data, size_t len) {
    if (!is_connected_ || sock_fd_ < 0) return -1;

    int ret = send(sock_fd_, data, len, 0);
    if (ret < 0) {
        perror("[TcpClient] Send failed");
        // 发送失败通常意味着连接断了
        closeConnection();
    }
    return ret;
}

int TcpClient::readMsg(char* buffer, int max_len) {
    if (!is_connected_ || sock_fd_ < 0) return -1;

    int ret = recv(sock_fd_, buffer, max_len, 0);
    if (ret <= 0) {
        if (ret == 0) cout << "[TcpClient] Server closed connection." << endl;
        else perror("[TcpClient] Recv error");
        
        closeConnection();
        return -1;
    }
    return ret;
}

void TcpClient::closeConnection() {
    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
    }
    is_connected_ = false;
}

bool TcpClient::checkNetStatus(const string& test_ip, uint16_t port, int timeout_sec) {
    int temp_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (temp_fd < 0) return false;

    // 设置超时
    setSocketTimeout(temp_fd, timeout_sec);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    
    // IP转换
    if (inet_pton(AF_INET, test_ip.c_str(), &addr.sin_addr) <= 0) {
        close(temp_fd);
        return false;
    }

    // 尝试连接
    int ret = connect(temp_fd, (struct sockaddr *)&addr, sizeof(addr));
    
    close(temp_fd);

    return (ret == 0); // 0 表示成功，即有网
}
