#ifndef	__USART1_H
#define __USART1_H

int USART1_Init(const char* device);
void USART1_SendString(const char* string);
int USART_ReadString(char* buf, size_t n);

#endif
