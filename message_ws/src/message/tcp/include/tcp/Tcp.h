#ifndef	__TCP_H
#define __TCP_H


#include <iostream>
using namespace std;
#include <string>
#include <vector>


#define SERVER_PORT         1234     /* 服务器端口号 */
#define SERVER_IP           "120.77.0.8"
#define TCP_MAX_SIZE        1024
#define NET_CHECK_INTERVAL  3

typedef enum{
    GET_IP_FAILED = -1,
    CREATE_SOCKET_FAILED = -2
}TCP_SCONNECTION_STATUS;

class TcpClient {
public:
    TcpClient();
    ~TcpClient();

    /**
     * @brief 连接到服务器 (业务连接)
     * @param ip 服务器IP
     * @param port 服务器端口
     * @param timeout_sec 连接超时时间(秒)
     * @return true:成功, false:失败
     */
    bool connectToServer(const std::string& ip, uint16_t port, int timeout_sec = 3);

    /**
     * @brief 发送数据
     */
    int sendMsg(const std::string& data);
    int sendMsg(const char* data, size_t len);

    /**
     * @brief 接收数据
     */
    int readMsg(char* buffer, int max_len);

    /**
     * @brief 主动断开连接
     */
    void closeConnection();

    /**
     * @brief 检查当前对象是否处于连接状态
     */
    bool isConnected() const { return is_connected_; }

    // =========================================================
    // 静态方法：工具函数 (Unified Interface Logic)
    // 不需要实例化对象即可调用，专门用于检测外网或某个IP是否通畅
    // =========================================================
    static bool checkNetStatus(const std::string& test_ip = "223.5.5.5", uint16_t port = 53, int timeout_sec = 1);

private:
    int sock_fd_;           // 每个对象独立的 socket 句柄
    bool is_connected_;     // 连接状态标记
    
    // 内部通用函数：设置超时
    static void setSocketTimeout(int fd, int sec);
};

// int Tcp_Reconnect(int server_local_sockfd);

/* -function about server ----------------------------*/
int Tcp_Server_Init(uint16_t port);
int Tcp_Server_Send(int client_socket, const char* buf);
int Tcp_Server_Read(int client_socket, char* buf);
void Tcp_Server_Close(int client_socket);

#endif  // __TCP_H