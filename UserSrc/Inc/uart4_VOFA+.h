#ifndef uart4_VOFA_H
#define uart4_VOFA_H
#include "main.h"

#define CH_COUNT 19//通道数量
#define VOFA_Period 10
void LoadData(void);
extern float ISR_time_us;
extern uint8_t VOFA_dma_tx_buf[CH_COUNT*4+4];
extern unsigned short VOFA_On;       //启用VOFA观测
#endif
