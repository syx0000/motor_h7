/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "can_rv.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t stateChange = 0;


uint8_t TorqueTestFlag = 0;//测试转矩控制精度
uint8_t TorqueTestLoop = 3;

SweepSineGenerator id_sweep_gen;

uint8_t TxData[64] = {0x12, 0x34, 0x56, 0x78, 0xAA, 0xBB, 0xCC, 0xDD, 0x9A, 0xBC, 0xDE, 0xF1, 0x23, 0x45, 0x67, 0x89,
0x12, 0x34, 0x56, 0x78, 0xAA, 0xBB, 0xCC, 0xDD,0x9A, 0xBC, 0xDE, 0xF1, 0x23, 0x45, 0x67, 0x89,
0x12, 0x34, 0x56, 0x78, 0xAA, 0xBB, 0xCC, 0xDD, 0x9A, 0xBC, 0xDE, 0xF1, 0x23, 0x45, 0x67, 0x89,
0x12, 0x34, 0x56, 0x78, 0xAA, 0xBB, 0xCC, 0xDD,0x9A, 0xBC, 0xDE, 0xF1, 0x23, 0x45, 0x67, 0x89}; // 32位测试数据
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_TIM6_Init();
	MX_FDCAN1_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_USART2_UART_Init();
	MX_USART6_UART_Init();
	MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(EN_GATE_GPIO_Port,EN_GATE_Pin,GPIO_PIN_SET);//预驱使能
	HAL_Delay(100);
	//1. ADC使能
	EnableADC();
	//2. TIM1初始化
//	TIM1->BDTR |= 0x8000;//Enable main output
	TIM1->BDTR |= 0xC000;//Enable main output and Automatic output enable(AOE)
	TIM1->DIER |= TIM_DIER_UIE;           // enable update interrupt
	TIM1->CR1  |= TIM_CR1_UDIS;						//先不产生更新事件
	TIM1->CR1  |= 0x0001;//Enable Counter
	//3. TIM6初始化
	htim6.Instance->CR1 |= TIM_CR1_CEN;
	
	//4.电机及编码器参数初始化
	RS485DIR_TX;//发送状态
	HAL_Delay(1);
	HAL_UART_Transmit_DMA(&huart2,USART2_TX_BUF,1);
	HAL_Delay(1);
	Motor_Init();
	DWT_Init();
	Encoder_Init();

	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
	/*Flash参数读取*/
//	Write_MotorData();
	Read_MotorData();

	//5.三环PI参数初始化
	InitControllerParams(&controller);//初始化控制器参数，包括电流环PI参数 如果阻感参数加载不成功为0，则电流环PI参数也为0，无法使能电机
//	Pid.Init(p_position_loop_g, Position_P, Position_I, 0.0f, 209.4f, 0.0004f, 1.0f);//FOC位置环控制参数初始化
	Pid.Init(p_position_loop_g, Position_P, Position_I, 0.0f, 150.0f, 0.0004f, 1.0f);//FOC位置环控制参数初始化
	
	Pid.Init(p_velocity_loop_g, Velocity_P, Velocity_I, 0.0f, 45.0f, 0.0f, 1.0f);//FOC速度环控制参数初始化 加入转速前馈 126-40.2；90-17.2
	
//	TIM1->CR1 ^= TIM_CR1_UDIS;//开始产生TIM1更新事件
	CalcCurrentOffset(&p_motor_g->phase_a_current_offset,&p_motor_g->phase_b_current_offset,&p_motor_g->phase_c_current_offset);//计算电流偏置
	
	
	/*上电自动回零*/
//	FSMstate = HOMING_MODE;
//	state_change = 1;
	
	
	

	TIM1->CR1 ^= TIM_CR1_UDIS;//开始产生TIM1更新事件
	EnterMenuState();
	
// 1. 初始化发生器
    
    // 设置参数：
    // - 幅值: 10A (根据你的电机调整，建议从额定电流的20%开始)
    // - 最大频率: 1000Hz (根据你的电流环期望带宽设置，通常设为期望带宽的2-5倍)
    // - 扫频时间: 2.0秒 (频率从0到1000Hz线性增加)
    // - 控制周期: 0.0001秒 (10kHz控制频率，dt = 1/10000)
//	SweepSine_Init(&id_sweep_gen, 24.3f, 400.0f, 5.0f, PeriodPWM);
    SweepSine_Init(&id_sweep_gen, 2.4324473f, 1500.0f, 10.0f, PeriodPWM);
    
// 2. 启动扫频
    SweepSine_Start(&id_sweep_gen);
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/*转矩控制精度测试*/
		if (p_motor_g->controlMode == FOC_CURRENT_LOOP && TorqueTestFlag == 1 && TorqueTestLoop != 0 && FSMstate == MOTOR_MODE)
		{
			for (uint8_t n=1;n<11;n++)
			{
				Motor_Iq = 24.32447f*n/10.0f;
				HAL_Delay(5000);
			}
			TorqueTestLoop--;
			if (TorqueTestLoop==0) Motor_Iq=0;
		}
		
		if (u8_100usFlag == 1)//100us时基
		{
			Pack_ActiveReport_Current(FOC_CAN_TxData ,p_motor_g->Q_axis_current_filt);
			CAN_SendMessage(CanID_Upload,FOC_CAN_TxData,8);//测试上传CAN ID = 0x700
			
			u8_100usFlag = 0;
		}
			
		if (u8_1msFlag == 1)//1ms时基
		{
			Calc_current_rms();
			TemperatureSample();
			
			static uint8_t cnt=0;
			if (fabs(p_position_loop_g->target - p_encoder2_g->pos_abs) < 0.015)
			//if (fabs(Motor_P - p_encoder2_g->pos_abs) < 0.015)
			{
				if (cnt < 20)
					cnt ++;
				if (cnt == 10)
				{
					bTargetPosFinish = true;
				}
			}
			else
			{
				cnt = 0;
				bTargetPosFinish = false;
			}
			
			if (VOFA_On)
			{
				LoadData();
				HAL_UART_Transmit_DMA(&huart6,VOFA_dma_tx_buf,4*CH_COUNT+4);
			}
			
			if (bDynamMode == true)
			{
				Pack_ActiveReport(FOC_CAN_TxData,p_encoder2_g->pos_abs,p_encoder2_g->mech_vel,p_motor_g->Q_axis_current_filt*KT_OUT,p_motor_g->Err1,p_motor_g->Err2,p_motor_g->Warning);
				CAN_SendMessage(CanID_Upload,FOC_CAN_TxData,16);//测试上传CAN ID = 0x7FE
			}
			
			u8_1msFlag = 0;
		}

		// Flash写入标志检查（主循环执行，避免ISR阻塞）
		if (flash_write_pending == 1)
		{
			flash_write_pending = 0;
			Write_MotorData();
		}

		if (stateChange==1)
		{
			FSMstate = MOTOR_MODE;
			state_change = 1;
			stateChange = 0;
		}
		if (stateChange==2)
		{
			EnablePWM();
			ApplyVoltDQToSVPWM(1.5, 0, PI/2.0f);
			HAL_Delay(1000);
			stateChange = 0;
			DisablePWM();
		}

//		if (HAL_GPIO_ReadPin(K1_GPIO_Port,K1_Pin) == GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
//		{
////		caliOn_flag = 1;
////		p_encoder_g->cali_start = 1;
////		Write_MotorData();
//			/*fdcan发送测试*/
//			TxHeader.Identifier = 0x12; // 设置报文ID
//			TxHeader.IdType = FDCAN_STANDARD_ID; //标准ID
//			TxHeader.TxFrameType = FDCAN_DATA_FRAME;//数据帧
//			TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 使用宏定义数据长度
//			TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//			TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 打开比特率切换
//			TxHeader.FDFormat = FDCAN_CLASSIC_CAN; // FDCAN帧格式
//			TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//			TxHeader.MessageMarker = 0;
//			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {
//				// 错误处理
//			}
			 /*询问编码器数据*/
//			RS485DIR_TX;
//			HAL_UART_Transmit_DMA(&huart2,USART2_TX_BUF,1);
//			HAL_Delay(1);
////		HAL_UART_Transmit_IT(&huart8,&UART8_TX_BUF,1);
//		 }

		/*验证CRC校验*/
//				uint8_t data[4] = {0x24,0xB0,0x46,0x40};
//				USART2_RX_DATA[4] = CalcCRC(data,4);

			 /*周期询问编码器数据-28.8us*/
//			 RS485DIR_TX;
//			 HAL_UART_Transmit_DMA(&huart2,USART2_TX_BUF,1);
//			 HAL_Delay(50);

			/*测试接收后回传数据*/
//			if (USART2_RX_COMPLETE == 1)
//			{
//				RS485DIR_TX;
//				HAL_UART_Transmit_DMA(&huart2,USART2_RX_BUF,8);
//				HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (unsigned char *)USART2_RX_BUF, USART2_REC_LEN);//开启空闲中断并启用DMA接收
//				USART2_RX_COMPLETE = 0;
//			}
			
			/*485发送测试 2.5M可正常发送*/
//			if (RS485_TXcnt)
//			{
//				uint8_t rs485buf_Tx[8]={0x01,0x03,0x00,0x00,0x00,0x08,0x44,0x0C};
//				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);	
//				HAL_UART_Transmit(&huart2,rs485buf_Tx,8,1000);
//				HAL_Delay(1);
//				RS485_TXcnt--;
//			}
			/*fdcan发送测试*/
//			TxHeader.Identifier = 0x12; // 设置报文ID
//			TxHeader.IdType = FDCAN_STANDARD_ID; //标准ID
//			TxHeader.TxFrameType = FDCAN_DATA_FRAME;//数据帧
//			TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 使用宏定义数据长度
//			TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//			TxHeader.BitRateSwitch = FDCAN_BRS_ON; // 打开比特率切换
//			TxHeader.FDFormat = FDCAN_FD_CAN; // FDCAN帧格式
//			TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//			TxHeader.MessageMarker = 0;
//			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {
//				// 错误处理
//			}
//			HAL_Delay(1000);

//		if (VOFA_cnt==VOFA_Period&&VOFA_On)//发送三个变量总线用时约170us，所以两个周期发送一次 这里需要注意的一点，如果发送间隔小于实际发送所有变量所需时间时，可能会导致转速环下转速跳变...
//		{
//			LoadData();
//			HAL_UART_Transmit_DMA(&huart8,VOFA_dma_tx_buf,4*CH_COUNT+4);
//			VOFA_cnt=0;
//		}

		if (caliOn_flag == 1)
		{
//			CalcCurrentOffset(&p_motor_g->phase_a_current_offset,&p_motor_g->phase_b_current_offset);//计算电流偏置
			EnablePWM();
			/*定位到电角度为0*/
//			ApplyVoltDQToSVPWM(1.5f, 0.0f, 0.0f);
//			HAL_Delay(2000);
//			DisablePWM();
//			caliOn_flag=0;
			
			/*测试硬件过流*/
//			p_motor_g->phase_order = POSITIVE_PHASE_ORDER;
//			float v_d = 0.0f;
//			for (uint32_t i=0;i<6000;i++)
//			{
//				v_d += 0.01f;
//				ApplyVoltDQToSVPWM(v_d, 0, 0);
//				HAL_Delay(1);
//			}
//			HAL_Delay(99);
//			DisablePWM();
//			caliOn_flag=0;
//			state_change = 0;
			
			/*编码器校准*/
			if (p_encoder_g->cali_start == 1)
			{
				p_encoder_g->cali_start = 0;
				Calibrate();//整定过程中电流波形周期变化
//				OrderPhases();//相序检测
			}
			/*电阻电感整定*/
			if (p_motor_g->cali_start == 1)
			{
				p_motor_g->cali_start = 0;
				Motor.MeasureResistance();
				Motor.MeasureInductance();
			}
			DisablePWM();

			/*校准后Flash参数写入（仅校准成功时）*/
			if (p_encoder_g->cali_finish == 1 || p_motor_g->motor_calibrated == 1)
			{
				Write_MotorData();
			}

			printf("\n\r Calibration complete.  Press 'esc' to return to menu\n\r");
			caliOn_flag = 0;
			state_change = 0;
		}
		ErrorReport();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 100;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 5;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  DisablePWM(); // 关闭PWM，防止电机失控
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
