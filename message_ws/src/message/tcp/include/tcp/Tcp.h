#ifndef	__TCP_H
#define __TCP_H


#include <iostream>
using namespace std;
#include <string>
#include <vector>


#define SERVER_PORT     1234     /* 服务器端口号 */
#define SERVER_IP       "120.77.0.8"
#define TCP_MAX_SIZE    1024

// int Tcp_Reconnect(int server_local_sockfd);

/* -function about server ----------------------------*/
int Tcp_Server_Init(uint16_t port);
int Tcp_Server_Send(int client_socket, const char *buf, int len);
int Tcp_Server_Send(int client_socket, const std::string &data);
int Tcp_Server_Read(int client_socket, char* buf);
void Tcp_Server_Close(int client_socket);
/* -function about client ---------------------------*/
int Tcp_Client_Init(uint16_t port, const char* IP);
int Tcp_Client_Send(const char* buf);
int Tcp_Client_Read(char* buf);
void Tcp_Client_Close(void);

#endif