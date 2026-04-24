#ifndef __FLASH_H
#define __FLASH_H
/* ------------------------------ Includes ------------------------------ */
#include "main.h"
/* ------------------------------ Defines ------------------------------ */
// 使用Bank2的Sector7，地址范围0x081E0000 - 0x081FFFFF (128KB)
#define DATA_FLASH_BANK    FLASH_BANK_2
#define DATA_FLASH_SECTOR  FLASH_SECTOR_7
#define DATA_FLASH_ADDR    0x081E0000 // Bank2的Sector7的起始地址
extern uint32_t flash_buffer[32]; // 32 * 4 = 128 Bytes


HAL_StatusTypeDef Flash_Init(void);
HAL_StatusTypeDef Flash_EraseSector(uint32_t Sector, uint32_t Bank);
void Write_MotorData(void);
void Read_MotorData(void);

uint32_t float2uint(float var);
float uint2float(uint32_t var);


#endif

