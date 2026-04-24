#include "Diag.h"
#include "main.h"

#define TEMP_MOSWARNING		90.0f
#define TEMP_MOSOver		100.0f//单位：℃
#define TEMP_MOTORWARNING	90.0f
#define TEMP_MOTOROver		100.0f//单位：℃

uint8_t Report_SWOvercur = 0;
uint8_t Report_HWOvercur = 0;
uint8_t Report_SWUndervolt = 0;
uint8_t Report_SWOvervolt = 0;
uint8_t Report_MOS_OverTEMP = 0;
uint8_t Report_MOTOR_OverTEMP = 0;
uint8_t Report_OverSpeed = 0;


float a_cur[3] = {0.0f},b_cur[3] = {0.0f},c_cur[3] = {0.0f},DC_bus[3] = {0.0f},temperature_MOS[3] = {0.0f},temperature_MOTOR[3] = {0.0f};
float a_cur_record = 0.0f,b_cur_record = 0.0f,c_cur_record = 0.0f,DC_bus_record = 0.0f,temperature_MOS_record = 0.0f,temperature_MOTOR_record = 0.0f;
float A_CURRENT = 0.0f,B_CURRENT = 0.0f,C_CURRENT = 0.0f;
float DC_BUS = 0.0f;
float TEMPERATURE_MOSFET = 0.0f,TEMPERATURE_MOTOR = 0.0f;
void errorDiag(void)
{
//	if (p_encoder_g->mech_vel > w_max|p_encoder_g->mech_vel < w_min)
//	{
//		disablePWM();
//		Report_OverSpeed = 1;
//	}
	
/*软件过流保护*/
	static uint8_t SWOvercur_cnt = 0;
	static uint8_t indexCur = 0;
//	static float A_CURRENT,B_CURRENT,C_CURRENT;
	a_cur[indexCur] = p_motor_g->phase_a_current;
	b_cur[indexCur] = p_motor_g->phase_b_current;
	c_cur[indexCur] = p_motor_g->phase_c_current;
	A_CURRENT = (a_cur[0] + a_cur[1] + a_cur[2]) / 3.0f;
	B_CURRENT = (b_cur[0] + b_cur[1] + b_cur[2]) / 3.0f;
	C_CURRENT = (c_cur[0] + c_cur[1] + c_cur[2]) / 3.0f;
	indexCur++;
	if (indexCur >= 3) indexCur = 0;
	if ( ( fabsf(A_CURRENT) > I_SWOver) || ( fabsf(B_CURRENT) > I_SWOver) || ( fabsf(C_CURRENT) > I_SWOver) )
	{
		SWOvercur_cnt++;
		if (SWOvercur_cnt > 10)
		{
			disablePWM();
			p_motor_g->Err1 |= MotorErr1_PhaseOverCurrent;
			p_motor_g->error = SwOverCurrent;
			p_motor_g->lastError = SwOverCurrent;//记录故障类型
			a_cur_record = A_CURRENT;
			b_cur_record = B_CURRENT;
			c_cur_record = C_CURRENT;
			Report_SWOvercur = 1;
		}
	}
	else 
	{
		SWOvercur_cnt = 0;
		if (p_motor_g->error==SwOverCurrent) p_motor_g->error=Normal;
	}
/*过压欠压保护*/
	static uint8_t SWOvervolt_cnt = 0;
	static uint8_t SWUndervolt_cnt = 0;
	static uint8_t indexDC_bus = 0;
//	static float DC_BUS;
	DC_bus[indexDC_bus] = p_motor_g->vbus;
	DC_BUS = (DC_bus[0] + DC_bus[1] + DC_bus[2]) / 3.0f;
	indexDC_bus++;
	if (indexDC_bus >= 3) indexDC_bus = 0;
	if (DC_BUS>60)//36-60
	{
		SWOvervolt_cnt++;
		if (SWOvervolt_cnt > 3)
		{
			disablePWM();
			p_motor_g->Err1 |= MotorErr1_OverVolt;
			p_motor_g->error=OverVolt;
			p_motor_g->lastError = OverVolt;//记录故障类型
			DC_bus_record = DC_BUS;
			Report_SWOvervolt = 1;
		}
	}
	else if (DC_BUS<24)//36-60
	{
		SWUndervolt_cnt++;
		if (SWUndervolt_cnt > 3)
		{
			disablePWM();
			p_motor_g->Err2 |= MotorErr2_LowVotage;
			p_motor_g->error=UnderVolt;
			p_motor_g->lastError = UnderVolt;//记录故障类型
			DC_bus_record = DC_BUS;
			Report_SWUndervolt = 1;
		}
	}
	else
	{
		SWOvervolt_cnt = 0;
		SWUndervolt_cnt = 0;
		if (p_motor_g->error==OverVolt||p_motor_g->error==UnderVolt) p_motor_g->error=Normal;
	}		

/*MOS过温保护*/
	static uint8_t TempWarning_MOS_cnt = 0,OverTEMP_MOS_cnt = 0;
	static uint8_t indexTEMP_MOS = 0;
//	static float TEMPERATURE_MOSFET;
	temperature_MOS[indexTEMP_MOS] = TEMP_MOS_filter1;
	TEMPERATURE_MOSFET = (temperature_MOS[0] + temperature_MOS[1] + temperature_MOS[2]) / 3.0f;
	indexTEMP_MOS++;
	if (indexTEMP_MOS >= 3) indexTEMP_MOS = 0;
	
	if (TEMPERATURE_MOSFET > TEMP_MOSOver)
	{
		OverTEMP_MOS_cnt++;
		if (OverTEMP_MOS_cnt > 3)
		{
			disablePWM();
			p_motor_g->Err1 |= MotorErr1_MosOverT;

			p_motor_g->error = MOS_OverTEMP;
			p_motor_g->lastError = MOS_OverTEMP;//记录故障类型
			temperature_MOS_record = TEMPERATURE_MOSFET;
			Report_MOS_OverTEMP = 1;
		}
	}
	else if (TEMPERATURE_MOSFET > TEMP_MOSWARNING)
	{
		TempWarning_MOS_cnt++;
		if (TempWarning_MOS_cnt > 3)
		{
			p_motor_g->Warning |= MotorWarning_MosOverT;
		}
	}
	else if (TEMPERATURE_MOSFET < (TEMP_MOSOver-5.0))
	{
		OverTEMP_MOS_cnt = 0;
		if (p_motor_g->error==MOS_OverTEMP) p_motor_g->error=Normal;
	}
	else if (TEMPERATURE_MOSFET < (TEMP_MOSWARNING-5.0))
	{
		TempWarning_MOS_cnt = 0;
	}
	
/*电机过温保护*/
	static uint8_t TempWarning_MOTOR_cnt = 0,OverTEMP_MOTOR_cnt = 0;
	static uint8_t indexTEMP_MOTOR = 0;
//	static float TEMPERATURE_MOTOR;
	temperature_MOTOR[indexTEMP_MOTOR] = TEMP_MOTOR_filter2;
	TEMPERATURE_MOTOR = (temperature_MOTOR[0] + temperature_MOTOR[1] + temperature_MOTOR[2]) / 3.0f;
	indexTEMP_MOTOR++;
	if (indexTEMP_MOTOR >= 3) indexTEMP_MOTOR = 0;
	
	if (TEMPERATURE_MOTOR > TEMP_MOTOROver)
	{
		OverTEMP_MOTOR_cnt++;
		if (OverTEMP_MOTOR_cnt > 3)
		{
			disablePWM();

			p_motor_g->Err1 |= MotorErr1_MotorOverT;

			p_motor_g->error = MOTOR_OverTEMP;
			p_motor_g->lastError = MOTOR_OverTEMP;//记录故障类型
			temperature_MOTOR_record = TEMPERATURE_MOTOR;
			Report_MOTOR_OverTEMP = 1;
		}
	}
	else if (TEMPERATURE_MOTOR > TEMP_MOTORWARNING)
	{
		TempWarning_MOTOR_cnt++;
		if (TempWarning_MOTOR_cnt > 3)
		{
			p_motor_g->Warning |= MotorWarning_MotorOverT;
		}
	}
	else if (TEMPERATURE_MOTOR < (TEMP_MOTOROver-5.0))
	{
		OverTEMP_MOTOR_cnt = 0;
		if (p_motor_g->error==MOTOR_OverTEMP) p_motor_g->error=Normal;
	}
	else if (TEMPERATURE_MOTOR < (TEMP_MOTORWARNING-5.0))
	{
		TempWarning_MOTOR_cnt = 0;
	}
}

void errorReport(void)
{
//	if (Report_SWOvercur||Report_HWOvercur||Report_SWOvervolt||Report_MOS_OverTEMP||Report_MOTOR_OverTEMP)//下电霍尔电流传感器输出低电压会误报软件过流Report_SWOvercur
	if (Report_HWOvercur||Report_SWOvervolt||Report_MOS_OverTEMP||Report_MOTOR_OverTEMP)//下电霍尔电流传感器输出低电压会误报软件过流Report_SWOvercur
	{
//		HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);//红灯
//		flash_reg[134] = p_motor_g->lastError;
//		Flash.Erase();//擦除FLASH
//		for (uint16_t i = 0; i<NUMBER_PARA; i++)  //保存140个参数
//		{
//			FLASH_ProgramWord(PARAM_FLASH_SECTOR+4*i, flash_reg[i]);
//		}
	}
	if (Report_SWOvercur==1)
	{
		printf("Software overcurrent,%.1f,%.1f,%.1f",a_cur_record,b_cur_record,c_cur_record);
		HAL_Delay(200);
		Report_SWOvercur = 0;
	}			
	if (Report_HWOvercur==1)
	{
		printf("Hardware overcurrent,%.1f,%.1f,%.1f",a_cur_record,b_cur_record,c_cur_record);
		HAL_Delay(200);
		Report_HWOvercur = 0;
	}	
	if (Report_SWOvervolt==1) 
	{
		printf("Over voltage,%.1f",DC_bus_record);
		HAL_Delay(200);
		Report_SWOvervolt = 0;
	}	
	if (Report_SWUndervolt==1)
	{
		printf("Under voltage,%.1f",DC_bus_record);
		HAL_Delay(200);
		Report_SWUndervolt = 0;
	}
	if (Report_MOS_OverTEMP==1)
	{
		printf("MOS over temperature,%.1f",temperature_MOS_record);
		HAL_Delay(200);
		Report_MOS_OverTEMP = 0;
	}
	if (Report_MOTOR_OverTEMP==1)
	{
		printf("MOTOR over temperature,%.1f",temperature_MOTOR_record);
		HAL_Delay(200);
		Report_MOTOR_OverTEMP = 0;
	}
	if (Report_OverSpeed==1)
	{
		printf("MOTOR over speed,%.1f",p_encoder_g->mech_vel);
		HAL_Delay(200);
		Report_OverSpeed = 0;
	}
}



