#ifndef	__UART_H
#define __UART_H

int UART1_Init(const char* device, int *fd);
void UART_SendData(int fd, char* string, size_t n);

#endif
