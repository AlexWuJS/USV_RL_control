#include <stdio.h>
#include <stdlib.h>
#include "usart/UART.h"
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/**
  * @brief  
  * @param  BaudRate: bo te lv.
  *     @arg B9600: 
  *     @arg B115200: 
* @retval 
  */
int UART1_Init(const char* device, int *fd)
{	
    	struct termios new_cfg = {0};
    // read and write enable.
    //close(
    *fd = open(device, O_RDWR | O_NOCTTY);
    if (-1 == (*fd))
    {
	char error_string[100] = {'\0'};
    	sprintf(error_string, "Device(%s) open failed", device);
        perror(error_string);
        return -1;
	}

	// set original mode
	cfmakeraw(&new_cfg);
	// reviece enable
	new_cfg.c_cflag |= CREAD;
	new_cfg.c_cflag |= CLOCAL;	
	//set bote
	cfsetspeed(&new_cfg, B115200);
	// set size of data
	new_cfg.c_cflag &= ~CSIZE;	//what
	new_cfg.c_cflag |= CS8;
	//no checkout
	new_cfg.c_cflag &= ~CSTOPB;
	new_cfg.c_iflag &= ~INPCK;
	// set stop bit
	new_cfg.c_cflag &= ~CSTOPB;
	// set MIN and TIME
	new_cfg.c_cc[VTIME] = 0;
	new_cfg.c_cc[VMIN] = 0;
	
	// clear buf
	if (-1 == tcflush(*fd, TCIOFLUSH))
	{
		perror("Clear buf failed");
		return -1;
	}
	if (-1 == tcsetattr(*fd, TCSANOW, &new_cfg))
	{
		perror("set UART failed");
		return -1;
	}
	return 0;
}

void UART_SendData(int fd, char* string, size_t n)
{
	ssize_t WriteBytes;
	
	WriteBytes = write(fd, string, n);
	if (-1 == WriteBytes)
	{
		perror("Data send failed");
	}
	else
		printf("Send string:%s\r\n", string);	
}

