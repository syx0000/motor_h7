#include "encoderRaw.h"
#include "FOC.h"
#include "main.h"

int32_t encoder1_raw_16 = 0;
int32_t encoder1_raw;
int32_t encoder1_raw_several_times_before;
int32_t encoder1_raw_filt = 0;

int32_t encoder2_raw;

/*T_Method*/
volatile uint8_t T_measurement_active = 0; //T法转速解算是否正在进行
volatile uint32_t start_time_CNT_T = 0; // 起始时间
volatile uint32_t start_encoder_cnt_T = 0; // 起始位置
volatile uint32_t current_time_CNT_T = 0; // 结束时间
volatile uint32_t current_encoder_cnt_T = 0;//结束位置

/*M_Method*/
volatile uint32_t start_time_CNT_M = 0; // 起始时间
volatile uint32_t start_encoder_cnt_M = 0; // 起始位置
volatile uint32_t current_encoder_cnt_M = 0;// 结束位置

void ReadEncoderRaw(void)
{
//		__asm volatile("cpsid i");
//		uint16_t txData = 0xffff;
//		uint16_t data_frame = 0x0000;         // 空数据帧（NOP）
//		uint16_t rxData = 0;
//		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_RESET);
//		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txData, (uint8_t*)&rxData, 1, 100);
//		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_SET);
//		for (int i = 0; i < 250; i++) __asm volatile("NOP");//软件延时520ns
//		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_RESET);
//		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&data_frame, (uint8_t*)&rxData, 1, 100);
//		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin,GPIO_PIN_SET);
//		encoder1_raw=(rxData&0x3fff);//偶校验+错误码+14bit编码器值
//		__asm volatile("cpsie i");
}

// 偶校验计算函数
uint16_t calc_even_parity(uint16_t data) {
    uint8_t count = 0;
    for (uint8_t i = 0; i<16; i++) {
        if (data & (1<<i)) count++;
    }
    return (count % 2) ? 0 : 1;
}

