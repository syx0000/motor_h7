#ifndef __encoderRaw_H
#define __encoderRaw_H

/* ------------------------------ Includes ------------------------------ */
#include "main.h"
/* ------------------------------ Defines ------------------------------ */
void ReadEncoderRaw(void);
void encoderConfig(void);
/* ------------------------------ Enum Typedef ------------------------------ */
extern int32_t encoder1_raw;
extern int32_t encoder1_raw_several_times_before;
extern int32_t encoder1_raw_16;
extern int32_t encoder1_raw_filt;
extern int32_t encoder2_raw;
//extern uint16_t SPI1_txBuffer[1];
//extern uint16_t SPI1_rxBuffer[1];
extern volatile uint32_t start_time_CNT_T;
extern volatile uint32_t start_time_CNT_M;
extern volatile uint32_t start_encoder_cnt_T;
extern volatile uint32_t current_time_CNT_T;
extern volatile uint32_t current_encoder_cnt_T;
extern volatile uint8_t T_measurement_active;
extern volatile uint32_t start_encoder_cnt_M;// 起始位置
extern volatile uint32_t current_encoder_cnt_M;// 结束位置

uint16_t calc_even_parity(uint16_t data);
/* ------------------------------ Struct Typedef ------------------------------ */

#endif
