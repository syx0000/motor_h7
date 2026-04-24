/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_hal_fdcan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t EncoderCnt = 0;
uint8_t VOFA_cnt = 0;
uint8_t Encoder_readFlag = 0;
float frequency = 100.0f;
float amplitude = 5.0f;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t SweepSineStart = 0;
uint32_t Sweep_step_count = 0;
float Sweep_time_elapsed = 0.0f;

uint8_t IDLE_flag = 0;
uint32_t angleInner;
uint32_t angleOutter;
float angleInnerFloat;
float angleOutterFloat;

uint32_t u32Timecnt;
uint8_t u8_100usFlag;
uint8_t u8_1msFlag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */
//	RS485DIR_RX; 
  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
	//10k开关频率
	HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_SET);
	ISR_start = DWT_CYCCNT;      // 记录起始计数值
	
	u8_100usFlag = 1;
	
	u32Timecnt++;
	if(u32Timecnt >= PWM_FREQUENCY_DEFAULT/1000)//1ms
	{
		u32Timecnt = 0;
		u8_1msFlag = 1;
	}

	RS485DIR_TX;
	startJADC();//开始ADC转换 必须保证开始转换在最前面进行(低电阻采样)！
//	ISR_start = DWT_CYCCNT;      // 记录起始计数值
	HAL_UART_Transmit_DMA(&huart2,USART2_TX_BUF,1);
//	ISR_end = DWT_CYCCNT;        // 记录结束计数值
//	ISR_time_us = ((float)(ISR_end - ISR_start))*1000000.0f/MCU_SYSCLK;    // 计算消耗的时间（us）
	errorDiag();//故障诊断
	
	encoderSample();//编码器采样
	voltageSample();//电压采样
	currentSample();//电流采样

	
//	ISR_end = DWT_CYCCNT;        // 记录结束计数值
//	ISR_time_us = ((float)(ISR_end - ISR_start))*1000000.0f/MCU_SYSCLK;    // 计算消耗的时间（us）
	if (TIM1->SR & TIM_SR_UIF )//Update interrupt flag为1
	{
//		static uint16_t IRQ_count;
////		static uint16_t IRQ_count2;
//		IRQ_count++;
////		IRQ_count2++;
//		if(IRQ_count==5000)
//		{
//			HAL_GPIO_TogglePin(LED_RUN_GPIO_Port,LED_RUN_Pin);
//			IRQ_count=0;
//		}
//		if(IRQ_count2==1)
//		{
//			RS485DIR_TX;
//			HAL_UART_Transmit_DMA(&huart2,USART2_TX_BUF,1);
//			IRQ_count2=0;
//		}
		
	
		float PP_position;  //PP修改版临时位置变量
		/// Check state machine state, and run the appropriate function
		switch(FSMstate)
		{
			case REST_MODE:// Do nothing
				if(state_change)
				{
					enter_menu_state();
				}
			break;
			
			case CALIBRATION_MODE:// Run encoder calibration procedure
				if(state_change)
				{
					if(p_motor_g->cali_start == 1||p_encoder_g->cali_start == 1)//电机参数辨识或者编码器整定
						caliOn_flag = 1;
					
					state_change = 0;
				}
			break;

			case MOTOR_MODE:// Run torque control
				if(state_change)//如果是第一次进电机模式（首次进入意味着刚切换到电机模式，还未发送运动指令）
				{
					if(p_encoder_g->cali_finish != 1)//未校准，拒绝进入MOTOR_MODE
					{
						FSMstate = REST_MODE;
						state_change = 0;
						break;
					}
//					p_position_loop_g->target = p_encoder_g->pos_abs;//当前位置值赋给指令值，以免位置环下刚进入电机模式，电机便会运动到0（p_position_loop_g->target初始化一般为0）			
					p_position_loop_g->target = p_encoder2_g->pos_abs;
					p_motor_g->i_d_ref = 0;/*电流环id指令清零*/
					p_motor_g->i_q_ref = 0;/*电流环iq指令清零*/
					p_velocity_loop_g->targetend = 0;/*速度环指令清零*/
					p_velocity_loop_g->target = 0;/*速度环指令清零*/
					Motor_P = p_encoder2_g->pos_abs;
					Motor_W = 0;
					Motor_Iq = 0;
					traj_complete = 0;
					PD_FOC_clear();
					enablePWM();
					svpwm_on = 1;//打开SVPWM
					state_change = 0;
				}
				else
				{
					CAN_timeout++ ;
					if((CAN_timeout > CAN_TIMEOUT) && (CAN_TIMEOUT > 0))
					{
						p_position_loop_g->target = p_encoder2_g->pos_abs;//当前位置值赋给指令值，以免位置环下刚进入电机模式，电机便会运动到0（p_position_loop_g->target初始化一般为0）			
						p_motor_g->i_d_ref = 0;/*电流环id指令清零*/
						p_motor_g->i_q_ref = 0;/*电流环iq指令清零*/
						p_velocity_loop_g->targetend = 0;/*速度环指令清零*/
						p_velocity_loop_g->target = 0;/*速度环指令清零*/
						Motor_P = p_encoder2_g->pos_abs;
						Motor_W = 0;
						Motor_Iq = 0;
						traj_complete = 0;
						PD_FOC_clear();
					}
					switch(p_motor_g->controlMode)
					{
						//0：PD控制 3：电流环 2：速度环 1：位置环 4：PP模式
						case MIT_PD:
							torque_control(&controller);//计算iq_ref（所需的5个参数由CAN通信数据解码得出）
							CurrentLoop();//电流环
						break;
						case FOC_CURRENT_LOOP:
							if(SweepSineStart == 1)
							{
								p_motor_g->i_d_ref = SweepSine_Update_ISR(&id_sweep_gen);
								p_motor_g->i_q_ref = 0.0f; 
//								p_motor_g->i_d_ref = 0.0f;
//								p_motor_g->i_q_ref = SweepSine_Update_ISR(&id_sweep_gen); 
								
//								// 计算当前相位增量并积分
//								float phase_increment = 2.0f * PI * frequency * 0.0001f;
//								static float current_phase = 0.0f; 
//								current_phase += phase_increment;
//								// 保持相位在0~2π范围内
//								if (current_phase > 2.0f * PI) current_phase -= 2.0f * PI;
//								p_motor_g->i_d_ref = 0.0f;
//								p_motor_g->i_q_ref = amplitude * sinf(current_phase);

							}
							else
							{
//								p_motor_g->i_d_ref = Motor_Iq;
//								p_motor_g->i_q_ref = 0.0f; 
								p_motor_g->i_d_ref = 0.0f;
								p_motor_g->i_q_ref = Motor_Iq;
							}
							CurrentLoop();
						break;
						case FOC_VELOCITY_LOOP:
							if(vel_loop_flag==1)//vel_calc_period次电流环运行一次速度环
							{
								p_velocity_loop_g->targetend = Motor_W;
								vel_loop_flag = 0;
								VelocityLoop();
							}
							CurrentLoop();
						break;
						case FOC_POSITION_LOOP:
							if(pos_loop_flag==1)//pos_calc_period次速度环运行一次位置环
							{
								pos_loop_flag = 0;
								
								p_position_loop_g->target = Motor_P;
								PositionLoop();
								
							}
							if(vel_loop_flag==1)//vel_calc_period次电流环运行一次速度环
							{
								vel_loop_flag = 0;
								VelocityLoop();
							}
							CurrentLoop();
						break;
						case FOC_POSITION_LOOP_PP:
							if(pos_loop_flag==1)//pos_calc_period次速度环运行一次位置环
							{
								if(trajcplt)
								{
									trajectory_tim_count++;		//NUM_STEPS次取一次位置插值 5次为2ms规划一次
									if(trajectory_tim_count > NUM_STEPS - 1)
									{//只要小于规划次数，就一直赋值给目标位置
										trajectory_tim_count = 0;
										if(get_next_position(p_planner_s, STEP_PANNEL, &PP_position))
										{
											p_position_loop_g->target = PP_position;
											trace_task_complete = 0x00;
										}
										else
										{
											trajcplt = 0;
											trace_task_complete = 0x01;
											printf("[OK] Planner Successful\r\n");
										}
									}
								}
								PositionLoop();
								pos_loop_flag = 0;
							}
							if(vel_loop_flag==1)//vel_calc_period次电流环运行一次速度环
							{
								vel_loop_flag = 0;
								VelocityLoop();
							}
							CurrentLoop();
						break;
						default:
						break;
					}
				}
			break;
						
			case SETUP_MODE:
				if(state_change)
				{
					enter_setup_state();
					state_change = 0;
				}
			break;
						
			case ENCODER_MODE:
				if(state_change)
				{
//					printf("Ouput-end Mechanical Angle:  %frad   Motor-end Mechanical Angle:  %frad   Electrical Angle:  %frad    Raw:  %dcnt\n\r",p_my_configure->state.hy_rad_multiturn/36000.0f*PI_TIMES_2, p_encoder_g->pos_abs, Mod(p_encoder_g->elec_pos,0.0f,PI_TIMES_2), encoder1_raw);
					state_change = 0;
				}
			break;
				
			case HOMING_MODE:
				if(state_change)
				{
					p_motor_g->i_d_ref = 0;/*电流环id指令清零*/
					p_motor_g->i_q_ref = 0;/*电流环iq指令清零*/
					p_velocity_loop_g->targetend = 0;/*速度环指令清零*/
					p_velocity_loop_g->target = 0;/*速度环指令清零*/
					PD_FOC_clear();
					enablePWM();
					svpwm_on = 1;//打开SVPWM
					state_change = 0;
				}
				else
				{
						CurrentLoop();
						if(vel_loop_flag==1)//两次电流环运行一次速度环 10k-5k
						{
							vel_loop_flag = 0;
							VelocityLoop();
						}
						if(pos_loop_flag==1)//两次速度环运行一次位置环 5k-2.5k
						{
							p_position_loop_g->target = p_encoder2_g->mech_offset;
							pos_loop_flag = 0;
							Homing();
						}
				}
				if(fabs(p_encoder2_g->mech_offset - p_encoder2_g->mech_abs)<0.05) 
				{
					disablePWM();
					FSMstate = REST_MODE;
					state_change = 1;
				}
			break;
			
			default: 
			break;
		}
		/* USER CODE END TIM1_UP_IRQn 0 */
		HAL_TIM_IRQHandler(&htim1);
		/* USER CODE BEGIN TIM1_UP_IRQn 1 */

		/*VOFA+发送数据 使用DMA，三个变量约2.2us*/
//		VOFA_cnt++;
//		if(VOFA_cnt==VOFA_Period&&VOFA_On)//发送三个变量总线用时约170us，所以两个周期发送一次 这里需要注意的一点，如果发送间隔小于实际发送所有变量所需时间时，可能会导致转速环下转速跳变...
//		{
//			LoadData();
//	//		ISR_start = DWT_CYCCNT;      // 记录起始计数值
//			HAL_UART_Transmit_DMA(&huart6,VOFA_dma_tx_buf,4*CH_COUNT+4);
//	//		ISR_end = DWT_CYCCNT;        // 记录结束计数值
//	//		ISR_time_us = ((float)(ISR_end - ISR_start))*1000000.0f/MCU_SYSCLK;    // 计算消耗的时间（us）
//			VOFA_cnt=0;
//		}
		
		TIM1->SR = 0x0; // reset the status register
	}
	ISR_end = DWT_CYCCNT;        // 记录结束计数值
	ISR_time_us = ((float)(ISR_end - ISR_start))*1000000.0f/MCU_SYSCLK;    // 计算消耗的时间（us）
	HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_RESET);
/* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
/* USER CODE BEGIN USART1_IRQn 0 */
	if(USART1->ISR & USART_ISR_RXNE_RXFNE)//接收中断
	{
		/* USER CODE END USART1_IRQn 0 */
		HAL_UART_IRQHandler(&huart1);
		/* USER CODE BEGIN USART1_IRQn 1 */
		char c = USART1_aRxBuffer[0];
		USART1_RX_BUF[USART1_RX_STA&0x7FFF]= USART1_aRxBuffer[0]; //接收数据转存
		USART1_RX_STA++;
		if(USART1_RX_STA>(USART1_REC_LEN-1))USART1_RX_STA=0;//接收数据错误,重新开始接收
	
		if(c == 27)//ESC
		{
			FSMstate = REST_MODE;
			state_change = 1;
			char_count = 0;
			cmd_id = 0;
			for(int i = 0; i<8; i++)
			{
				cmd_val[i] = 0;
			}
		}
		if(FSMstate == REST_MODE)
		{
			switch (c)
			{
				case 'c':
					FSMstate = CALIBRATION_MODE;
					printf("\n\r e-Encoder");
					delay_us(10);
					printf("\n\r m-Motor");
					delay_us(10);
					printf("\n\r h-HALL Amplitude");
					delay_us(10);
					printf("\n\r a-All");
					delay_us(10);
				break;

				case 'm':
					FSMstate = MOTOR_MODE;
					state_change = 1;
					printf("\n\r %-6s %-40s %-10s %-10s %-2s\n\r\n\r", "prefix", "parameter", "min", "max", "current value");
					delay_us(10);
					printf(" %-6s %-40s %-10s %-10s %.1f\n\r", "i", "Motor Current iq(A)", "-50.0", "50.0", Motor_Iq);
					delay_us(10);
					printf(" %-6s %-40s %-10s %-10s %.1f\n\r", "v", "Module Output Mechanical Angular Velocity(rad/s)", "-20.0", "20.0", Motor_W/GR);
					delay_us(10);
					printf(" %-6s %-40s %-10s %-10s %.1f\n\r", "p", "Module Output Mechanical Angular(rad)", "-50000.0", "50000.0", Motor_P/GR);
					delay_us(10);
					printf("\n\r To change a value, type 'prefix''value''ENTER'\n\r i.e. 'v10''ENTER'\n\r\n\r");
					delay_us(10);
				break;

				case 'e':
					FSMstate = ENCODER_MODE;
					state_change = 1;
				break;

				case 's':
					FSMstate = SETUP_MODE;
					state_change = 1;
				break;
				
				case 'h':
					FSMstate = HOMING_MODE;
					state_change = 1;
				break;

				case 'z':
					p_encoder2_g->mech_offset = p_encoder2_g->mech_abs;
					Write_MotorData();
					printf("\n\r  Saved new zero position:  %.4f\n\r\n\r",p_encoder2_g->mech_offset);
	//				enc_sincos_read_deg(p_my_configure);  //末端角度计算
	//				p_encoder2_g->mech_offset = p_my_configure->state.hy_rad_multiturn/36000.0f*PI_TIMES_2*GR;//单位：rad/28
	//				flash_reg[135] = float2uint(p_encoder2_g->mech_offset);			
	//				Flash.Erase();//擦除FLASH	
	//					
	//				for(uint16_t i=0; i<NUMBER_PARA; i++)  //保存140个参数
	//				{
	//					FLASH_ProgramWord(PARAM_FLASH_SECTOR+4*i, flash_reg[i]);
	//				}
	//				printf("\n\r  Saved new zero position:  %.4f\n\r\n\r", p_encoder2_g->mech_offset);
		  
				break;
			}
		}
		else if(FSMstate == SETUP_MODE)
		{
			if(c == 13)//回车
			{					
				switch (cmd_id)
				{
					case 'c':
						switch (atoi(cmd_val))
						{
							case MIT_PD:
								p_motor_g->controlMode = MIT_PD;
								printf("MIT_PD");
								break;
							case FOC_CURRENT_LOOP:
								p_motor_g->controlMode = FOC_CURRENT_LOOP;
								printf("FOC_CURRENT_LOOP");
								break;
							case FOC_VELOCITY_LOOP:
								p_motor_g->controlMode = FOC_VELOCITY_LOOP;
								printf("FOC_VELOCITY_LOOP");
								break;
							case FOC_POSITION_LOOP:
								p_motor_g->controlMode = FOC_POSITION_LOOP;
								printf("FOC_POSITION_LOOP");
								break;
							case FOC_POSITION_LOOP_PP:
								p_motor_g->controlMode = FOC_POSITION_LOOP_PP;
								printf("FOC_POSITION_LOOP_PP");
								break;
							default: 
								break;
						}
						break;
					case 'b':
						I_BW_set = atof(cmd_val);
						I_BW = fmaxf(fminf(atof(cmd_val), 2000.0f), 10.0f);
						controller.ki_d = I_BW*2.0f*PI*p_motor_g->phase_resistance*(1/PWM_FREQUENCY_DEFAULT);//积分参数：电流环带宽*相电阻*电流环周期
						controller.ki_q = I_BW*2.0f*PI*p_motor_g->phase_resistance*(1/PWM_FREQUENCY_DEFAULT);
						controller.k_d = I_BW*2.0f*PI*p_motor_g->phase_inductance;//比例参数：电流环带宽*相电感
						controller.k_q = I_BW*2.0f*PI*p_motor_g->phase_inductance;
						if(I_BW_set>2000.0f) printf("I_BW set max %.1f.  Press 'esc' to return to menu or continue to set parameter.\n\r",I_BW);
						else if(I_BW_set<100.0f) printf("I_BW set min %.1f.  Press 'esc' to return to menu or continue to set parameter.\n\r",I_BW);
						else	printf("I_BW set succeed,%.1f.  Press 'esc' to return to menu or continue to set parameter.\n\r",I_BW);
						break;
					
					case 'i':
						CAN_ID = atoi(cmd_val);
						printf("CAN_ID set succeed,%d.  Press 'esc' to return to menu or continue to set parameter.\n\r",CAN_ID);
						break;
					
					case 'm':
						CAN_MASTER = atoi(cmd_val);
						printf("CAN_MASTER set succeed,%d.  Press 'esc' to return to menu or continue to set parameter.\n\r",CAN_MASTER);
						break;
					
					case 'o':
						I_SWOver_set=atof(cmd_val);
						I_SWOver = fmaxf(fminf(atof(cmd_val), 50.0f), 0.0f);
						if(I_SWOver_set>50.0f) printf("I_SWOver set max %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",I_SWOver);
						else if(I_SWOver_set<0.0f) printf("I_SWOver set min %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",I_SWOver);
						else printf("I_SWOver set succeed,%.1f.  Press 'esc' to return to menu or continue to set parameter.\n\r",I_SWOver);
						break;
									
					case 'P':
						Velocity_P_set=atof(cmd_val);
						Velocity_P = fmaxf(fminf(atof(cmd_val), 1.0f), 0.0f);
						p_velocity_loop_g->kp = Velocity_P;
						if(Velocity_P_set>1.0f) printf("Velocity_P set max %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Velocity_P);
						else if(Velocity_P_set<0.0f) printf("Velocity_P set min %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Velocity_P);
						else	printf("Velocity_P set succeed,%.7f.  Press 'esc' to return to menu or continue to set parameter.\n\r",Velocity_P);
						break;
									
					case 'I':
						Velocity_I_set=atof(cmd_val);
						Velocity_I = fmaxf(fminf(atof(cmd_val), 1.0f), 0.0f);
						p_velocity_loop_g->ki = Velocity_I;
						if(Velocity_I_set>1.0f) printf("Velocity_I set max %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Velocity_I);
						else if(Velocity_I_set<0.0f) printf("Velocity_I set min %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Velocity_I);
						else	printf("Velocity_I set succeed,%.7f.  Press 'esc' to return to menu or continue to set parameter.\n\r",Velocity_I);
						break;
										
					case 'M':
						Position_P_set=atof(cmd_val);
						Position_P = fmaxf(fminf(atof(cmd_val), 100.0f), 0.0f);
						p_position_loop_g->kp = Position_P;
						if(Position_P_set>100.0f) printf("Position_P set max %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Position_P);
						else if(Position_P_set<0.0f) printf("Position_P set min %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Position_P);
						else	printf("Position_P set succeed,%.7f.  Press 'esc' to return to menu or continue to set parameter.\n\r",Position_P);
						break;
									
					case 'N':
						Position_I_set=atof(cmd_val);
						Position_I = fmaxf(fminf(atof(cmd_val), 1.0f), 0.0f);
						p_position_loop_g->ki = Position_I;
						if(Position_I_set>1.0f) printf("Position_I set max %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Position_I);
						else if(Position_I_set<0.0f) printf("Position_I set min %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Position_I);
						else	printf("Position_I set succeed,%.7f.  Press 'esc' to return to menu or continue to set parameter.\n\r",Position_I);
						break;
					/*电流环PI参数*/
					case 'Q':
						Current_P_set=atof(cmd_val);
						Current_P = fmaxf(fminf(atof(cmd_val), 100.0f), 0.0f);
						controller.k_d = Current_P;
						controller.k_q = Current_P;
						if(Current_P_set>100.0f) printf("Current_P set max %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Current_P);
						else if(Current_P_set<0.0f) printf("Current_P set min %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Current_P);
						else	printf("Current_P set succeed,%.7f.  Press 'esc' to return to menu or continue to set parameter.\n\r",Current_P);
						break;
									
					case 'W':
						Current_I_set=atof(cmd_val);
						Current_I = fmaxf(fminf(atof(cmd_val), 1.0f), 0.0f);
						controller.ki_d = Current_I;
						controller.ki_q = Current_I;
						if(Current_I_set>1.0f) printf("Current_I set max %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Current_I);
						else if(Current_I_set<0.0f) printf("Current_I set min %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",Current_I);
						else	printf("Current_I set succeed,%.7f.  Press 'esc' to return to menu or continue to set parameter.\n\r",Current_I);
						break;
									
					case 'A':
						FOC_velAccDec_set=atof(cmd_val);
						FOC_velAccDec = fmaxf(fminf(atof(cmd_val), 5000.0f), 0.0f);
						if(FOC_velAccDec_set>1.0f) printf("FOC_velAccDec set max %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",FOC_velAccDec);
						else if(FOC_velAccDec_set<0.0f) printf("FOC_velAccDec set min %.1f.  Press 'esc' to return to menu or continue to set parameter\n\r",FOC_velAccDec);
						else	printf("FOC_velAccDec set succeed,%.5f.  Press 'esc' to return to menu or continue to set parameter.\n\r",FOC_velAccDec);
						break;
					case 't':
						CAN_TIMEOUT = atoi(cmd_val);
						break;
					case 'r':
						if(atoi(cmd_val)==0&&p_motor_g->error==HwOverCurrent) p_motor_g->error=Normal;
						break;							 
			   
					default:
						printf("\n\r '%c' Not a valid command prefix\n\r\n\r", cmd_id);
						break;
				}
				char_count = 0;
				for(uint8_t i=0;i<8;i++) cmd_val[i]=0;
			}
			else
			{
				if(char_count == 0)
				{
					cmd_id = c;
				}
				else
				{
					cmd_val[char_count-1] = c;   
				}
				char_count++;
			}
		}
		else if(FSMstate == MOTOR_MODE)
		{
			if(c == 13)//回车
			{
				switch (cmd_id)
				{
					case 'i':
	//					Motor_Iq_set=atof(cmd_val);
						Motor_Iq = fmaxf(fminf(atof(cmd_val), iq_max), iq_min);
	//					if(Motor_Iq_set>iq_max) printf("Motor_Iq set max %.1f.  Press 'esc' to return to menu\n\r",Motor_Iq);
	//					else if(Motor_Iq_set<iq_min) printf("Motor_Iq set min %.1f.  Press 'esc' to return to menu\n\r",Motor_Iq);
	//					else	printf("Motor_Iq set succeed,%.1f.  Press 'esc' to return to menu or continue to set parameter.\n\r",Motor_Iq);
	//					printf("%.1f\n\r",Motor_Iq);
						break;									
					case 'v':
	//					Motor_W_set=atof(cmd_val) * GR;
						Motor_W = fmaxf(fminf(atof(cmd_val), w_max), w_min) * GR;
	//					if(Motor_W_set>w_max) printf("Motor_W set max %.1f.  Press 'esc' to return to menu\n\r",Motor_W/GR);
	//					else if(Motor_W_set<w_min) printf("Motor_W set min %.1f.  Press 'esc' to return to menu\n\r",Motor_W/GR);
	//					else	printf("Motor_W set succeed,%.1f.  Press 'esc' to return to menu or continue to set parameter.\n\r",Motor_W/GR);
	//					printf("%.1f\n\r",Motor_W);
						//之所以不再打印，是因为这个会导致发送速度指令0（速度指令切换...）之后产生大电流，具体原因待评估※！！！！！！！！！！？？？？？？？
						break;
	//				case 'p':
	////				Motor_P_set=atof(cmd_val);
	//					Motor_P = fmaxf(fminf(atof(cmd_val), p_max), p_min);
	//								
	//					/*位置插补*/
	////				if(trace_task_complete)
	//					{
	//						p_data_soft_num = 0;  
	//						trajectory_g.pos1 = p_encoder_g->pos_abs;			
	//						trajectory_g.pos2 = Motor_P;	//rad
	////					trajectory_g.acc =  500;      //rad/s/s
	////					trajectory_g.dec =  500;	    //rad/s/s
	////					trajectory_g.vel_max = 104.72;    //rad/s
	//						trajectory_g.acc =  500;      //rad/s/s
	//						trajectory_g.dec =  500;	    //rad/s/s
	//						trajectory_g.vel_max = 104.72;    //rad/s
	//						Trajectory.Get_Trape_Para(p_trajectory_g);
	//						Trajectory.Get_Trape_Plan(p_trajectory_g);
	//					}
	////				for(uint16_t i = 0; i <= trajectory_g.p_data_max_num; i++)
	////				{							
	////					p_position_loop_g->target = trajectory_g.pos_demand[i];
	////					p_velocity_loop_g->targetend = trajectory_g.vel_demand[i];
	////					p_motor_g->i_q_ref  = trajectory_g.acc_demand[i];
	//////					printf("%.4f\n\r",p_position_loop_g->target);					
	////					printf("%d,%f,%f,%f\r\n",i,p_position_loop_g->target,p_velocity_loop_g->targetend,p_motor_g->i_q_ref);
	////					HAL_Delay(1);
	////				}
	//								
	////				if(Motor_P_set>p_max) printf("Motor_P set max %.1f.  Press 'esc' to return to menu\n\r",Motor_P);
	////				else if(Motor_P_set<p_min) printf("Motor_P set min %.1f.  Press 'esc' to return to menu\n\r",Motor_P);
	////				else	printf("Motor_P set succeed,%.1f.  Press 'esc' to return to menu or continue to set parameter.\n\r",Motor_P);
	//					printf("%.1f\n\r",Motor_P);
	//					break;  								
					case 'p':
						Motor_P = fmaxf(fminf(atof(cmd_val), p_max), p_min) + p_encoder2_g->mech_offset;//在机械零位的基础上运动
						/*位置插补*/
						if(p_motor_g->controlMode == FOC_POSITION_LOOP_PP)
						{
							init_planner(p_planner_s, p_encoder2_g->pos_abs, Motor_P, 8.376f, 2.0f, 2.0f); //目标位置、最大速度、加速度、减速度需要按照需求设置
							trajcplt = 1;
						}
						printf("%.1f\n\r",Motor_P);
						break;
					default:
						printf("\n\r '%c' Not a valid command prefix\n\r\n\r", cmd_id);
						break;
				}
				char_count = 0;
				for(uint8_t i=0;i<8;i++) cmd_val[i]=0;
			}
			else
			{
				if(char_count == 0)
				{
					cmd_id = c;
				}
				else
				{
					cmd_val[char_count-1] = c;   
				}
				char_count++;
			}
		}
		else if(FSMstate == CALIBRATION_MODE)
		{
			if(c == 13)//回车
			{
				switch (cmd_id)
				{
					case 'e':
						p_encoder_g->cali_start = 1;
						state_change = 1;
						break;									
					case 'm':
						p_motor_g->cali_start = 1;
						state_change = 1;
						break;	
					case 'a':
						p_encoder_g->cali_start = 1;
						p_motor_g->cali_start = 1;
						state_change = 1;
						break;					
					default:
						printf("\n\r '%c' Not a valid command prefix\n\r\n\r", cmd_id);
						break;
				}
			}
			else
			{
				cmd_id = c;
			}
		}
		c = 0;
		USART1_aRxBuffer[0] = 0;
		
		HAL_UART_Receive_IT(&huart1,(unsigned char *)USART1_aRxBuffer, USART1_RXBUFFERSIZE);
	}
	else HAL_UART_IRQHandler(&huart1);//其他中断
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE))//空闲中断
	{
		IDLE_flag = 1;
//		huart2.gState = HAL_UART_STATE_READY;
		HAL_UART_IRQHandler(&huart2);
		USART2->ICR  |= 0x00000010;//向此位写入“1”时， USART_ISR 寄存器中的 IDLE 标志将清零 进中断后先清标志位
//		HAL_UARTEx_ReceiveToIdle_IT(&huart2, (unsigned char *)USART2_RX_BUF, USART2_REC_LEN);//开启空闲中断
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (unsigned char *)USART2_RX_BUF, USART2_REC_LEN);//开启空闲中断
	}
	else if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC))//发送完成中断
	{
		RS485DIR_RX;                        //发送完毕 改成接收状态
		HAL_UART_IRQHandler(&huart2);
		USART2->ICR  |= 0x00000040;//向此位写入“1”时， USART_ISR 寄存器中的 TC 标志将清零
	}
	else
	/* USER CODE END USART2_IRQn 0 */
	HAL_UART_IRQHandler(&huart2);
	/* USER CODE BEGIN USART2_IRQn 1 */
	
	/* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2&&IDLE_flag==1)
	{
	//	USART2_RX_COMPLETE = 1;//告诉主循环有数据待处理
		USART2_RX_Cnt = Size;
		IDLE_flag = 0;
		HAL_UART_DMAStop(huart);// 停止当前 DMA 传输（防止数据覆盖）
		/*可以在这里将缓存USART2_RX_BUF中的数据转移到缓存USART2_RX_DATA中，然后清空USART2_RX_BUF，后面直接处理缓存USART2_RX_DATA*/
		for(uint8_t i=0;i<USART2_RX_Cnt;i++)
		{
			USART2_RX_DATA[i] = USART2_RX_BUF[i];
			USART2_RX_BUF[i] = 0;
		}
		angleInner = USART2_RX_DATA[2] << 16 | USART2_RX_DATA[1] << 8 | USART2_RX_DATA[0];
		angleOutter = USART2_RX_DATA[5] << 16 | USART2_RX_DATA[4] << 8 | USART2_RX_DATA[3];
	//	angleInnerFloat = angleInner / (float)(1 << 24) * 360;
	//	angleOutterFloat = angleOutter / (float)(1 << 24) *360;
		if (USART2_RX_DATA[7] != calcCRC(USART2_RX_DATA,7)) EncoderCnt++;
	}
}
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
//    // 检查是否发生溢出错误
//    if (huart->ErrorCode & HAL_UART_ERROR_DMA) {
//        // 必须清除溢出错误标志（ORE）
//        // 对于STM32H7系列，通常使用以下宏
//        __HAL_UART_CLEAR_OREFLAG(huart);
//        
//        // 清除错误代码
//        huart->ErrorCode = HAL_UART_ERROR_NONE;
//        
//        // 重新启动DMA接收。这是恢复通信的重要一步
//        // 假设你的接收句柄是`huart1`，缓冲区是`RxBuffer`
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (unsigned char *)USART2_RX_BUF, USART2_REC_LEN);//开启空闲中断
//    }
//    
//    // 建议也检查并处理其他可能错误，如帧错误、噪声错误等
//    // if (huart->ErrorCode & (HAL_UART_ERROR_FE | HAL_UART_ERROR_NE)) {
//    //     ... // 清除相应标志
//    //     HAL_UART_Receive_DMA(huart, (uint8_t*)RxBuffer, BUFFER_SIZE);
//    // }
//}
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {//上电后会报错，要重新初始化...
//    if (huart->Instance == USART2) {
////        // 可选：清除错误标志，如溢出错误
////        __HAL_UART_CLEAR_OREFLAG(huart);
////        // 重新启动接收
////        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (unsigned char *)USART3_RX_BUF, USART3_REC_LEN);
////			MX_USART2_UART_Init();
//			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (unsigned char *)USART2_RX_BUF, USART2_REC_LEN);//开启空闲中断并启用DMA接收
//		}
//}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (hfdcan->Instance == FDCAN1)
	{
		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
		{//检查是否是"新报文"中断
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, FDCAN1_RX_BUF) == HAL_OK)//1. 调用函数，获取报文头和数据
			{
				CAN_MsgProcess(RxHeader.Identifier, FDCAN1_RX_BUF);
			}
		}
	}
}
/* USER CODE END 1 */
