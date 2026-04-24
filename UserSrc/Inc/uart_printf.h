#ifndef _UART_PRINT_H
#define _UART_PRINT_H
 
#include "main.h"
#include "stdio.h"
 
 
#ifndef _UART_PRINT_C
//放置需要被外部调用的uart.c文件中的全局变量\函数
 
#endif
extern UART_HandleTypeDef huart1;
 
void Usart_SendString(uint8_t *str);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
 
 
#endif

