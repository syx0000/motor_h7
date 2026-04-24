/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define USART1_RXBUFFERSIZE 1 //每次接收1个数据进入一次中断
#define USART1_REC_LEN 200  	 //定义最大接收字节数 200

#define UART8_REC_LEN 20  	 //定义最大接收字节数
#define USART2_REC_LEN 20  	 //定义最大接收字节数
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern unsigned short USART1_RX_STA;       //接收状态标记
extern unsigned char  USART1_aRxBuffer[USART1_RXBUFFERSIZE];//HAL库使用的串口接收缓冲
extern unsigned char  USART1_RX_BUF[USART1_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.

extern unsigned char UART8_RX_COMPLETE;
extern unsigned char UART8_TX_BUF[];
extern unsigned char UART8_RX_BUF[UART8_REC_LEN];
extern unsigned char USART2_RX_COMPLETE;
extern unsigned char USART2_RX_Cnt;//记录一次接收的数量
extern uint8_t USART2_TX_BUF[1];
extern unsigned char USART2_RX_BUF[USART2_REC_LEN];
extern unsigned char USART2_RX_DATA[USART2_REC_LEN];//应用缓冲区
#define RS485DIR_TX HAL_GPIO_WritePin(RS485_DIR_GPIO_Port,RS485_DIR_Pin,GPIO_PIN_SET);//发送状态
#define RS485DIR_RX HAL_GPIO_WritePin(RS485_DIR_GPIO_Port,RS485_DIR_Pin,GPIO_PIN_RESET);//接收状态
#define UART8_TX_POSITION 0x43
#define UART8_TX_TEMP 0x74
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

