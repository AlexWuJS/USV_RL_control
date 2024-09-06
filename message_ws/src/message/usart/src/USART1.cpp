#include <stdio.h>
#include <unistd.h>
#include "usart/UART.h"
#include "string.h"


static int fd;

/**
  * @brief  
  * @param  BaudRate: bo te lv.
  *     @arg B9600: 
  *     @arg B115200: 
* @retval 
  */
int USART1_Init(const char* device)
{
	// UART config
	return UART1_Init(device, &fd);
}

void USART1_SendString(const char* string)
{
	ssize_t WriteBytes;
	
	WriteBytes = write(fd, string, strlen(string));
	if (-1 == WriteBytes)
	{
		perror("USART1 send failed");
	}
	// else
	// 	printf("USART1 send: %s\r\n", string);
}

int USART_ReadString(char* buf, size_t n)
{
	ssize_t ReadBytes;
	
	ReadBytes = read(fd, buf, n);
	if (-1 == ReadBytes)
	{
		perror("USART1 read failed");
		return -1;
	}
	
	return 0;
}
