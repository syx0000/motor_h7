#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32_hal_legacy.h"
#include "arm_math.h"
#include "flash.h"

// 定义一个32字节的缓冲区
uint32_t flash_buffer[32]; // 32 * 4 = 128 Bytes

HAL_StatusTypeDef Flash_Init(void)
{
    HAL_StatusTypeDef status;

    // 解锁Flash
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }

    // 清除所有错误标志（可选但推荐）
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);

    return HAL_OK;
}

HAL_StatusTypeDef Flash_EraseSector(uint32_t Sector, uint32_t Bank)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Banks = Bank;
    EraseInitStruct.Sector = Sector;
    EraseInitStruct.NbSectors = 1; // 每次只擦除一个扇区
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V-3.6V

    // 执行擦除操作
    return HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
}

void Write_MotorData(void)
{
	HAL_StatusTypeDef status;

	// 1. 准备数据
	flash_buffer[0] = float2uint(p_encoder_g->elec_offset);//电角度偏移
	flash_buffer[1] = float2uint(p_motor_g->phase_order);//相序
	flash_buffer[2] = p_encoder_g->cali_finish;//编码器整定完成标志
	flash_buffer[3] = float2uint(p_motor_g->phase_resistance);//电阻
	flash_buffer[4] = float2uint(p_motor_g->phase_inductance);//电感
	flash_buffer[5] = p_motor_g->motor_calibrated;//电机整定完成标志
	flash_buffer[6] = 0;//每次写入数据时会清零故障码
	flash_buffer[7] = float2uint(p_encoder2_g->mech_offset);//输出端零位机械偏移
	
	flash_buffer[8] = float2uint(p_min);//位置下限
	flash_buffer[9] = float2uint(p_max);//位置上限
	flash_buffer[10] = float2uint(w_min);//速度下限
	flash_buffer[11] = float2uint(w_max);//速度上限
	flash_buffer[12] = float2uint(iq_min);//扭矩下限
	flash_buffer[13] = float2uint(iq_max);//扭矩上限	
	flash_buffer[14] = float2uint(KP_MIN);//KP下限
	flash_buffer[15] = float2uint(KP_MAX);//KP上限
	
	flash_buffer[16] = float2uint(KD_MIN);//KD下限
	flash_buffer[17] = float2uint(KD_MAX);//KD上限
	flash_buffer[18] = FDCAN_ID;
	flash_buffer[19] = 0;
	flash_buffer[20] = 0;
	flash_buffer[21] = 0;
	flash_buffer[22] = 0;
	flash_buffer[23] = 0;
	
	flash_buffer[24] = 0;
	flash_buffer[25] = 0;
	flash_buffer[26] = 0;
	flash_buffer[27] = 0;
	flash_buffer[28] = 0;
	flash_buffer[29] = 0;
	flash_buffer[30] = 0;
	flash_buffer[31] = 0;

    // 2. 初始化Flash（解锁）
    status = Flash_Init();
    if (status != HAL_OK)
	{
        printf("Flash_Init Error!");
        return;
    }

    // 3. 擦除目标扇区
	status = Flash_EraseSector(DATA_FLASH_SECTOR, DATA_FLASH_BANK);
	if (status != HAL_OK)
	{
		HAL_FLASH_Lock(); // 擦除失败，重新锁定Flash
		printf("Flash_Erase Error!");
		return;
	}

    // 4. 只能以32字节（256bit对齐）为单位写入数据，如有新数据，需要再次调用该函数，并更换首地址
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, DATA_FLASH_ADDR, (uint32_t)(&flash_buffer[0]));
    if (status != HAL_OK)
	{
        HAL_FLASH_Lock(); // 写入失败，重新锁定Flash
		printf("Flash_Program Error!");
        return;
    }
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, DATA_FLASH_ADDR+32, (uint32_t)(&flash_buffer[8]));
    if (status != HAL_OK)
	{
        HAL_FLASH_Lock(); // 写入失败，重新锁定Flash
		printf("Flash_Program Error!");
        return;
    }
	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, DATA_FLASH_ADDR+64, (uint32_t)(&flash_buffer[16]));
    if (status != HAL_OK)
	{
        HAL_FLASH_Lock(); // 写入失败，重新锁定Flash
		printf("Flash_Program Error!");
        return;
    }

    // 5. 锁定Flash
    HAL_FLASH_Lock();
}
/*Flash参数读取*/
void Read_MotorData (void) 
{
	for(uint16_t i=0; i<32; i++)
		flash_buffer[i] = *(uint32_t*)(DATA_FLASH_ADDR + 4*i);
	
	p_encoder_g->cali_finish = flash_buffer[2];
	p_motor_g->motor_calibrated = flash_buffer[5];
	if(p_encoder_g->cali_finish == 1)
	{
		p_encoder_g->elec_offset = uint2float(flash_buffer[0]);//电角度偏移
		p_motor_g->phase_order = uint2float(flash_buffer[1]);//相序
	}
	if(p_motor_g->motor_calibrated == 1)
	{
		p_motor_g->phase_resistance = uint2float(flash_buffer[3]);//电阻
		p_motor_g->phase_inductance = uint2float(flash_buffer[4]);//电感
	}
	p_motor_g->lastError = flash_buffer[6];
	p_encoder2_g->mech_offset = uint2float(flash_buffer[7]);
	
	p_min = uint2float(flash_buffer[8]);
	p_max = uint2float(flash_buffer[9]);
	w_min = uint2float(flash_buffer[10]);
	w_max = uint2float(flash_buffer[11]);
	iq_min = uint2float(flash_buffer[12]);
	iq_max = uint2float(flash_buffer[13]);
	KP_MIN = uint2float(flash_buffer[14]);
	KP_MAX = uint2float(flash_buffer[15]);
	KD_MIN = uint2float(flash_buffer[16]);
	KD_MAX = uint2float(flash_buffer[17]);
}

typedef union
{
	float f;
	uint32_t da;
} UN32;

uint32_t float2uint(float var)
{	
	UN32 a;
	
	a.f = var;
	return a.da;
}

float uint2float(uint32_t var)
{
	UN32 a;
	
	a.da = var;
	return a.f;
}