#include "can_rv.h"
#include "FOC.h"
#include "main.h"
float KP_MIN=0.0f;
float KP_MAX=500.0f;
float KD_MIN=0.0f;
float KD_MAX=5.0f;
float POS_MIN=-5.0f;//机械角度
float POS_MAX=5.0f;
float SPD_MIN=-10.472f;//机械速度
float SPD_MAX=10.472f;
float T_MIN=-30.0f;
float T_MAX=30.0f;

#define I_MIN -50.0f
#define I_MAX 50.0f

bool bDynamMode = false;
bool bTargetPosFinish = false;
bool bPos_PP_Flag = false;

uint8_t FOC_CAN_TxData[16] = {0};
uint32_t CAN_timeout = 0;
uint32_t CAN_TIMEOUT = 0;//默认无心跳保护
char CommMode = CommResponseMode;//默认问答模式


/*位置插补*/
extern Trajectory_t trajectory_g;
extern Trajectory_t *p_trajectory_g;


// 发送CAN消息函数
void CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t len) 
{
	FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.Identifier = id; // 设置报文ID
	TxHeader.IdType = FDCAN_STANDARD_ID; //标准ID
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;//数据帧
	if(len == 8)
		TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 使用宏定义数据长度
	else if(len == 12)
		TxHeader.DataLength = FDCAN_DLC_BYTES_12; // 使用宏定义数据长度
	else if(len == 16)
		TxHeader.DataLength = FDCAN_DLC_BYTES_16; // 使用宏定义数据长度
	else
		TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 默认8字节
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON; // 打开比特率切换
	TxHeader.FDFormat = FDCAN_FD_CAN; // FDCAN帧格式
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK) {
		// 错误处理
	}	
//    FDCAN_TxHeaderTypeDef TxHeader;  // 定义 CAN 发送头
//    uint32_t TxMailbox;  					 // 邮箱标识
//    TxHeader.DataLength = len;                 // 数据长度，最大 8 字节
//    TxHeader.Identifier = id;                // 标准标识符
//    TxHeader.IdType = CAN_ID_STD;          // 使用标准 ID
//    TxHeader.TxFrameType = CAN_RTR_DATA;        // 数据帧
//    TxHeader.TransmitGlobalTime = DISABLE;  // 关闭全局时间戳
// 
//    // 发送 CAN 消息，使用 HAL 库提供的函数
//    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK) 
//		{
//        // 如果发送失败，调用错误处理函数
//        Error_Handler();
//    }
}
//配置过滤器，启动CAN外设和中断
void CAN_Config(void)
{
// 
//	CAN_FilterTypeDef  sFilterConfig;
// 
//  /*配置CAN过滤器*/
//  sFilterConfig.FilterBank = 0;                     //过滤器0
//  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//  sFilterConfig.FilterIdHigh = 0x0000;              //32位ID
//  sFilterConfig.FilterIdLow = 0x0000;
//  sFilterConfig.FilterMaskIdHigh = 0x0000;          //32位MASK
//  sFilterConfig.FilterMaskIdLow = 0x0000;
//  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;//过滤器0关联到FIFO0
//  sFilterConfig.FilterActivation = ENABLE;          //激活滤波器0
//  sFilterConfig.SlaveStartFilterBank = 14;
//	
//	
// if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig) != HAL_OK)//初始化过滤器
//	  {
//		Error_Handler();
//		}
// if(HAL_CAN_Start(&hcan1) != HAL_OK)//打开can
//	 {
//		Error_Handler();//这里会返回错误（硬件同步失败：CAN_MSR-INAK） 更换can芯片后正常
//		printf("CAN IC Damaged");
//	 }
// if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)//开启接受邮邮箱0挂起中断
//	 {
//		Error_Handler();
//	 }
}
//中断接收数据函数
void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_RxHeaderTypeDef *hfdcan1) 
{
//    CAN_RxHeaderTypeDef RxHeader;  // CAN 接收头
//    // 获取接收到的消息
//    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, CAN_RxData) != HAL_OK) 
//		{
//        // 如果接收失败，直接返回（避免执行后续无效操作）
//        Error_Handler();
//        return;
//		}		
//		
//		CAN_MsgProcess(&RxHeader,CAN_RxData);
		
//		/*PD控制通信协议*/
//		if(p_motor_g->controlMode == MIT_PD)
//		{
//			if( (RxHeader.StdId == 0x36) && (CAN_RxData[0] == CAN_ID) && (CAN_RxData[1] == 0x1C) )//切换控制模式
//			{
//				switch(CAN_RxData[2])
//				{
//						case MIT_PD:
//							p_motor_g->controlMode = MIT_PD; 
//						break;
//						case FOC_CURRENT_LOOP:
//							p_motor_g->controlMode = FOC_CURRENT_LOOP;
//						break;
//						case FOC_VELOCITY_LOOP:
//							p_motor_g->controlMode = FOC_VELOCITY_LOOP;
//						break;
//						case FOC_POSITION_LOOP:
//							p_motor_g->controlMode = FOC_POSITION_LOOP;
//						break;
//						default: 
//						break;
//				}
//			}
//			if((uint8_t)RxHeader.StdId == CAN_ID)
//			{
//					if(((CAN_RxData[0]==0xFF) && (CAN_RxData[1]==0xFF) && (CAN_RxData[2]==0xFF) && (CAN_RxData[3]==0xFF) && 
//							(CAN_RxData[4]==0xFF) && (CAN_RxData[5]==0xFF) && (CAN_RxData[6]==0xFF) && (CAN_RxData[7]==0xFC)))//FF FF FF FF FF FF FF FC
//					{
//							FSMstate = MOTOR_MODE;//切电机模式
//							state_change = 1;
//					}
//					else if(((CAN_RxData[0]==0xFF) && (CAN_RxData[1]==0xFF) && (CAN_RxData[2]==0xFF) && (CAN_RxData[3]==0xFF) && 
//							(CAN_RxData[4]==0xFF) && (CAN_RxData[5]==0xFF) && (CAN_RxData[6]==0xFF) && (CAN_RxData[7]==0xFD)))//FF FF FF FF FF FF FF FD
//					{
//							FSMstate = REST_MODE;//切休息模式
//							state_change = 1;
//					}
//					else if(((CAN_RxData[0]==0xFF) && (CAN_RxData[1]==0xFF) && (CAN_RxData[2]==0xFF) && (CAN_RxData[3]==0xFF) && 
//							(CAN_RxData[4]==0xFF) && (CAN_RxData[5]==0xFF) && (CAN_RxData[6]==0xFF) && (CAN_RxData[7]==0xFE)))//FF FF FF FF FF FF FF FE
//					{
//	//            PositionSensor_ZeroPosition();//设置当前位置为机械零位
//					}
//					else if(FSMstate == MOTOR_MODE)//80 00 A0 00 00 A0 08 F5   0,2.5,0,1.25,0.6  80 00 80 00 00 00 08 00 停止
//					{
//							unpack_cmd(CAN_RxData, &controller);//指令解码，得到PD控制所需的5个参数
//					}
//					PD_pack_reply(CAN_TxData, controller.theta_mech, controller.dtheta_mech, controller.i_q_filt*KT_OUT);//反馈位置及转矩信息
//					CAN_SendMessage(CAN_MASTER,CAN_TxData,6);
//			}
//		}
//		else
//		{
//					/*FOC闭环通信协议*/
//				switch(RxHeader.StdId)
//			{
//				uint32_t parameterSet;
//				float target;
//			//电机状态控制命令  1，电机失能；2，电机使能
//					case 0x38://帧ID
//						switch(CAN_RxData[CAN_ID])
//						{
//							case 0x01:
//								FSMstate = REST_MODE;//切休息模式
//								state_change = 1;
//							break;
//							case 0x02:
//								FSMstate = MOTOR_MODE;//切电机模式
//								state_change = 1;
//							break;
//							default: 
//							break;
//						}
//					break;

//					case 0x36://帧ID
//						if(CAN_RxData[0] == CAN_ID)
//						{
//							parameterSet = (CAN_RxData[5]<<24) | (CAN_RxData[4]<<16) | (CAN_RxData[3]<<8) | CAN_RxData[2];
//							switch(CAN_RxData[1])
//							{
//							case 0x1C://控制模式
//								if(FSMstate == REST_MODE)//电机必须为空闲状态
//								{
//									switch(parameterSet)
//									{
//										case MIT_PD:
//											p_motor_g->controlMode = MIT_PD;//CAN_ID 1C 01 00 00 00 00 00 
//										break;
//										case FOC_CURRENT_LOOP:
//											p_motor_g->controlMode = FOC_CURRENT_LOOP;//CAN_ID 1C 02 00 00 00 00 00 
//										break;
//										case FOC_VELOCITY_LOOP:
//											p_motor_g->controlMode = FOC_VELOCITY_LOOP;//CAN_ID 1C 03 00 00 00 00 00 
//										break;
//										case FOC_POSITION_LOOP:
//											p_motor_g->controlMode = FOC_POSITION_LOOP;//CAN_ID 1C 04 00 00 00 00 00 
//										break;
//										default: 
//										break;
//									}
//								}

//							break;

//							default: 
//							break;
//							}
//						
//						}
//					break;				

//					case 0x32://帧ID
//						if(FSMstate == MOTOR_MODE)//电机必须为使能状态
//						{
//							target =  (int16_t)((CAN_RxData[2*CAN_ID]<<8) | CAN_RxData[2*CAN_ID+1])/100.0f;
//							switch(p_motor_g->controlMode)
//							{
//								case FOC_CURRENT_LOOP:
//									Motor_Iq = fmaxf(fminf(target, iq_max), iq_min);//电流限幅
//								break;
//								case FOC_VELOCITY_LOOP:
//									Motor_W = fmaxf(fminf(target, w_max), w_min);//转速限幅
//								break;
//								case FOC_POSITION_LOOP:									
//								
////									trajectory_g.pos1 = p_encoder_g->pos_abs;			
////									trajectory_g.pos2 = target;	//rad
////									trajectory_g.acc =  100;      //rad/s/s
////									trajectory_g.dec =  100;	    //rad/s/s
////									trajectory_g.vel_max = 52;    //rad/s
////									Trajectory.Get_Trape_Para(p_trajectory_g);
////									Trajectory.Get_Trape_Plan(p_trajectory_g);
//								
//									Motor_P = fmaxf(fminf(target, p_max), p_min);//位置限幅
//								break;
//								default: 
//								break;
//							}
//							target = 0;
//							FOC_pack_reply(FOC_CAN_TxData,p_encoder_g->mech_vel,p_motor_g->Q_axis_current,p_encoder_g->mech_pos,p_motor_g->vbus);
//							CAN_SendMessage(0x50+CAN_ID,FOC_CAN_TxData,8);
//					}
//					break;
//					case 0x33://帧ID
//						if(FSMstate == MOTOR_MODE)//电机必须为使能状态
//						{
//							target =  ((int16_t)((CAN_RxData[2*(CAN_ID-4)]<<8) | CAN_RxData[2*(CAN_ID-4)+1]))/100.0f;
//							switch(p_motor_g->controlMode)
//							{
//								case FOC_CURRENT_LOOP:
//									target = fmaxf(fminf(target, iq_max), iq_min);//电流限幅
//									Motor_Iq = target;
//								break;
//								case FOC_VELOCITY_LOOP:
//									target = fmaxf(fminf(target, w_max), w_min);//转速限幅
//									Motor_W  = target;
//								break;
//								case FOC_POSITION_LOOP:
//									target = fmaxf(fminf(target, p_max), p_min);//位置限幅
//									Motor_P  = target;
//								break;
//								default: 
//								break;
//							}
//							target = 0;
//							FOC_pack_reply(FOC_CAN_TxData,p_encoder_g->mech_vel,p_motor_g->Q_axis_current,p_encoder_g->mech_pos,p_motor_g->vbus);
//							CAN_SendMessage(0x50+CAN_ID,FOC_CAN_TxData,8);
//						}
//						
//					break;
//						
//					default: 
//						break;
//				}
//		}
}

void pack_reply(uint8_t *tData,float pos, float vel, float torque, uint16_t err1, uint8_t err2, uint8_t warning)
{
	uint8_t state = 0;
	
	uint32_t p_int = float_to_uint(pos, p_min, p_max, 24);
	uint16_t v_int = float_to_uint(vel/GR, w_min, w_max, 16);
	uint16_t t_int = float_to_uint(torque, iq_min, iq_max, 16);
	uint16_t temp_Motor = (uint16_t)(TEMPERATURE_MOTOR*10);
	uint16_t temp_Mos = (uint16_t)(TEMPERATURE_MOSFET*10);
		
	tData[0] = p_int;//位置
	tData[1] = p_int>>8;
	tData[2] = p_int>>16;
	tData[3] = v_int;//速度
	tData[4] = v_int>>8;
	tData[5] = t_int;//扭矩
	tData[6] = t_int>>8;

/*	tData[7] = err1&0x00FF;//一级故障-严重故障
	tData[8] = err1>>8;//一级故障-严重故障
	tData[9] = err2;//二级故障-一般故障
	tData[10] = warning;//三级故障-警告
*/
	tData[7] = temp_Motor;//电机温度
	tData[8] = temp_Motor>>8;
	tData[9] = temp_Mos;//Mos温度
	tData[10] = temp_Mos>>8;
	
	if(FSMstate == REST_MODE)
		state &= 0xFE;
	else
		state |= 0x01;
	
	if(err1!=0 || err2!=0)
		state |= 0x02;
	
	if(warning!=0)
		state |= 0x04;

	
	tData[11] = state;//电机运行状态
}

void Pack_ActiveReport_Current(uint8_t *tData ,float torque)
{
	uint8_t state = 0;
	
	uint32_t I_target = Motor_Iq*1000 + 50000;
	uint32_t I_real = torque*1000 + 50000;
		
	tData[0] = I_target;
	tData[1] = I_target>>8;
	tData[2] = I_target>>16;
	tData[3] = I_target>>24;
	
	tData[4] = I_real;
	tData[5] = I_real>>8;
	tData[6] = I_real>>16;
	tData[7] = I_real>>24;
}

void Pack_ActiveReport(uint8_t *tData,float pos, float vel, float torque, uint16_t err1, uint8_t err2, uint8_t warning)
{
	uint8_t state = 0;
	
//	uint32_t p_int = float_to_uint(pos, p_min, p_max, 24);
//	uint16_t v_int = float_to_uint(vel/GR, w_min, w_max, 16);
//	uint16_t t_int = float_to_uint(torque, iq_min, iq_max, 16);
//	uint16_t temp_Motor = (uint16_t)(TEMPERATURE_MOTOR*10);
//	uint16_t temp_Mos = (uint16_t)(TEMPERATURE_MOSFET*10);

	uint16_t t_int = (uint16_t)(p_motor_g->phase_a_Current_RMS*100);//相电流有效值，单位A
	// 修复溢出：先缩放范围再转换，避免乘法溢出
	uint16_t v_int = float_to_uint(vel/GR * 10.0f, w_min*10.0f, w_max*10.0f, 16);//速度，单位0.1RPM
	uint32_t p_int = float_to_uint(pos * 1000.0f, p_min*1000.0f, p_max*1000.0f, 24);//位置，单位0.001°
	
	uint16_t temp_Motor = (uint16_t)(TEMPERATURE_MOTOR*10);
	uint16_t temp_Mos = (uint16_t)(TEMPERATURE_MOSFET*10);
	
	tData[0] = t_int;//扭矩
	tData[1] = t_int>>8;
	
	tData[2] = v_int;//速度
	tData[3] = v_int>>8;
	
	tData[4] = p_int;//位置
	tData[5] = p_int>>8;
	tData[6] = p_int>>16;
	tData[7] = p_int>>24;

	tData[8] = temp_Motor;//电机温度
	tData[9] = temp_Motor>>8;
	tData[10] = temp_Motor;//Mos温度
	tData[11] = temp_Motor>>8;
	
	if(FSMstate == REST_MODE)
		state &= 0xFE;
	else
		state |= 0x01;
	
	if(err1!=0 || err2!=0)
		state |= 0x02;
	
	if(warning!=0)
		state |= 0x04;
	
	if(bTargetPosFinish == true)
		state |= 0x08;
	else
		state &= 0xF7;
	
	tData[12] = state;//电机运行状态
	
	tData[13] = 0;
	tData[14] = 0;
	tData[15] = 0;
}

void unpack_speed_cmd(uint8_t CAN_RxData[])
{
	uint16_t v_raw = ((CAN_RxData[FDCAN_ID*3-2]&0xFF)<<8)|(CAN_RxData[FDCAN_ID*3-3]);
	float temp = uint32_to_float(v_raw, w_min, w_max, 16);
	Motor_W = temp * GR;//下发的速度指令是输出角速度，电机端指令速度需要乘以减速比
	
	p_motor_g->controlMode = FOC_VELOCITY_LOOP;//设置速度模式
}

void unpack_torque_cmd(uint8_t CAN_RxData[])
{
	uint16_t t_raw = ((CAN_RxData[FDCAN_ID*3-2]&0xFF)<<8)|(CAN_RxData[FDCAN_ID*3-3]);
	Motor_Iq = uint32_to_float(t_raw, iq_min, iq_max, 16)/KT_OUT;
	
	p_motor_g->controlMode = FOC_CURRENT_LOOP;//设置扭矩模式
}

void unpack_position_cmd(uint8_t CAN_RxData[])
{
	uint32_t p_raw = (CAN_RxData[FDCAN_ID*6-4]<<16)|(CAN_RxData[FDCAN_ID*6-5]<<8)|(CAN_RxData[FDCAN_ID*6-6]);
	uint16_t v_raw = ((CAN_RxData[FDCAN_ID*6-2])<<8)|(CAN_RxData[FDCAN_ID*6-3]);
	
	Motor_P = (uint32_to_float(p_raw, p_min, p_max, 24)) + p_encoder2_g->mech_offset;
	
	if(p_motor_g->controlMode == FOC_POSITION_LOOP_PP && trajcplt == 0)
	{
		init_planner(p_planner_s, p_encoder2_g->pos_abs, Motor_P, 8.376f, 2.0f, 2.0f); //目标位置、最大速度、加速度、减速度需要按照需求设置
		trajcplt = 1;
	}

	p_position_loop_g->output_limit = (uint32_to_float(v_raw, w_min, w_max, 16)) * GR;//位置模式时，速度指令用于限制运行转速

	if(bPos_PP_Flag != true)//非位置PP模式
		p_motor_g->controlMode = FOC_POSITION_LOOP;//设置位置模式
}

void unpack_MIT_cmd(uint8_t CAN_RxData[])
{
	uint32_t p_raw = (CAN_RxData[FDCAN_ID*12-10]<<16)|(CAN_RxData[FDCAN_ID*12-11]<<8)|(CAN_RxData[FDCAN_ID*12-12]);
	uint16_t v_raw = ((CAN_RxData[FDCAN_ID*12-8])<<8)|(CAN_RxData[FDCAN_ID*12-9]);
	uint16_t t_raw = (CAN_RxData[FDCAN_ID*12-6]<<8)|(CAN_RxData[FDCAN_ID*12-7]);
	uint16_t kp_raw = (CAN_RxData[FDCAN_ID*12-4]<<8)|(CAN_RxData[FDCAN_ID*12-5]);
	uint16_t kd_raw = (CAN_RxData[FDCAN_ID*12-2]<<8)|(CAN_RxData[FDCAN_ID*12-3]);
	
	controller.p_des = uint32_to_float(p_raw, p_min, p_max, 24);
	controller.v_des = uint32_to_float(v_raw, w_min, w_max, 16) * GR;
	controller.t_ff = uint32_to_float(t_raw, iq_min, iq_max, 16);
	controller.kp = uint32_to_float(kp_raw, KP_MIN, KP_MAX, 16);
	controller.kd = uint32_to_float(kd_raw, KD_MIN, KD_MAX, 16);
	
	p_motor_g->controlMode = MIT_PD;//设置MIT模式
}

union RV_TypeConvert
{
	float to_float;
	int to_int;
	unsigned int to_uint;
	uint8_t buf[4];
}rv_type_convert;

union RV_TypeConvert2
{
	int16_t to_int16;
	uint16_t to_uint16;
	uint8_t buf[2];
}rv_type_convert2;

void CAN_MsgProcess(uint32_t Identifier, uint8_t *FDCANRxData)
{
	bool bSaveDataFlag = false;
	CAN_timeout = 0; // 收到CAN消息，清零超时计数器

	for(uint8_t i=0;i<64;i++)
	{
		FDCAN1_RX_DATA[i] = FDCANRxData[i];
		FDCANRxData[i] = 0;
	}
	if(Identifier==0x80+FDCAN_ID) 
	{
		//if(bDynamMode == false)
			CAN_SendMessage(FDCAN_ID+0x100,FDCAN1_TxData,12);//反馈状态(单轴)
	}
	
	if(Identifier==0x700+FDCAN_ID) //使能、失能、设置位置零点、清除错误
	{
		if(((FDCAN1_RX_DATA[0]==0xFF) && (FDCAN1_RX_DATA[1]==0xFF) && (FDCAN1_RX_DATA[2]==0xFF) && (FDCAN1_RX_DATA[3]==0xFF) && 
			(FDCAN1_RX_DATA[4]==0xFF) && (FDCAN1_RX_DATA[5]==0xFF) && (FDCAN1_RX_DATA[6]==0xFF) && (FDCAN1_RX_DATA[7]==0xFA)))
		{
			FSMstate = MOTOR_MODE;//使能
			state_change = 1;
		}
		else if(((FDCAN1_RX_DATA[0]==0xFF) && (FDCAN1_RX_DATA[1]==0xFF) && (FDCAN1_RX_DATA[2]==0xFF) && (FDCAN1_RX_DATA[3]==0xFF) && 
			(FDCAN1_RX_DATA[4]==0xFF) && (FDCAN1_RX_DATA[5]==0xFF) && (FDCAN1_RX_DATA[6]==0xFF) && (FDCAN1_RX_DATA[7]==0xFB)))
		{
			FSMstate = REST_MODE;//失能
			state_change = 1;
		}
		else if(((FDCAN1_RX_DATA[0]==0xFF) && (FDCAN1_RX_DATA[1]==0xFF) && (FDCAN1_RX_DATA[2]==0xFF) && (FDCAN1_RX_DATA[3]==0xFF) && 
			(FDCAN1_RX_DATA[4]==0xFF) && (FDCAN1_RX_DATA[5]==0xFF) && (FDCAN1_RX_DATA[6]==0xFF) && (FDCAN1_RX_DATA[7]==0xFC)))
		{
			p_encoder2_g->mech_offset = p_encoder2_g->mech_abs;//设置位置零点
			flash_write_pending = 1;
		}
		else if(((FDCAN1_RX_DATA[0]==0xFF) && (FDCAN1_RX_DATA[1]==0xFF) && (FDCAN1_RX_DATA[2]==0xFF) && (FDCAN1_RX_DATA[3]==0xFF) && 
			(FDCAN1_RX_DATA[4]==0xFF) && (FDCAN1_RX_DATA[5]==0xFF) && (FDCAN1_RX_DATA[6]==0xFF) && (FDCAN1_RX_DATA[7]==0xFD)))
		{
			p_motor_g->error = Normal;//清除错误
			p_motor_g->Err1 = MotorErr1_Nomal;
			p_motor_g->Err2 = MotorErr1_Nomal;
			p_motor_g->Warning = MotorWarning_Nomal;
		}
		
		pack_reply(FOC_CAN_TxData,p_encoder2_g->pos_abs,p_encoder2_g->mech_vel,p_motor_g->Q_axis_current_filt*KT_OUT,p_motor_g->Err1,p_motor_g->Err2,p_motor_g->Warning);
		//if(bDynamMode == false)
			CAN_SendMessage(0x100+FDCAN_ID,FOC_CAN_TxData,12);
	}
	if(Identifier==0x600+FDCAN_ID)//对象字典读写
	{
		uint16_t parameter_U16;
		uint32_t parameter_U32;
		uint32_t parameterIndex = FDCAN1_RX_DATA[1]|FDCAN1_RX_DATA[2]<<8;
		uint32_t parameterLength = FDCAN1_RX_DATA[3];
		switch(FDCAN1_RX_DATA[0])
		{
			case 0x40://对象字典读
			{
				switch(parameterIndex)
				{
					case 0x2000://位置范围下限
					{
						parameter_U32 = (uint32_t)p_min;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x2001://位置范围上限
					{
						parameter_U32 = (uint32_t)p_max;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x2002://速度范围下限
					{
						parameter_U32 = (uint32_t)w_min;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x2003://速度范围上限
					{
						parameter_U32 = (uint32_t)w_max;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x2004://扭矩范围下限
					{
						parameter_U32 = (uint32_t)iq_min;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x2005://扭矩范围上限
					{
						parameter_U32 = (uint32_t)iq_max;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x2006://扭矩刚度（产生单位转角（rad）所需施加的扭矩（N.m））下限
					{
						parameter_U32 = (uint32_t)KP_MIN;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x2007://扭矩刚度上限
					{
						parameter_U32 = (uint32_t)KP_MAX;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x2008://转动惯量阻尼/扭矩阻尼系数（产生单位角速度（rad/s）所需的阻尼扭矩（N.m））下限
					{
						parameter_U32 = (uint32_t)KD_MIN;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x2009://转动惯量阻尼/扭矩阻尼系数 上限
					{
						parameter_U32 = (uint32_t)KD_MAX;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x200A://单位：HZ，默认1000HZ  所有模式下的指令下发周期 （驱动器IP模式下备用，可忽略）
					{
						parameter_U16 = 1000;//(uint16_t)SyncCycle;
						FDCAN1_TxData[4] = parameter_U16&0xFF;
						FDCAN1_TxData[5] = (parameter_U16>>8)&0xFF;
						FDCAN1_TxData[6] = 0;
						FDCAN1_TxData[7] = 0;
					}
					break;
					case 0x200B://驱动器位置环KP（调试驱动器备用）
					{
						parameter_U32 = (uint32_t)Position_P;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x200C://驱动器速度环KP（调试驱动器备用）
					{
						parameter_U32 = (uint32_t)Velocity_P;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					case 0x200D://驱动器速度环Ki（调试驱动器备用）
					{
						parameter_U32 = (uint32_t)Velocity_I;
						FDCAN1_TxData[4] = parameter_U32&0xFF;
						FDCAN1_TxData[5] = (parameter_U32>>8)&0xFF;
						FDCAN1_TxData[6] = (parameter_U32>>16)&0xFF;
						FDCAN1_TxData[7] = (parameter_U32>>24)&0xFF;
					}
					break;
					default:
					break;
				}
				FDCAN1_TxData[0] = 0x60;
				FDCAN1_TxData[1] = FDCAN1_RX_DATA[1];
				FDCAN1_TxData[2] = FDCAN1_RX_DATA[2];
				FDCAN1_TxData[3] = FDCAN1_RX_DATA[3];
				//if(bDynamMode == false)
					CAN_SendMessage(FDCAN_ID+0x580,FDCAN1_TxData,8);
			}
			break;
			case 0x23://对象字典写
			{
				switch(parameterIndex)
				{
					case 0x2000://位置范围下限
					{
						p_min = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
						bSaveDataFlag = true;
					}
					break;
					case 0x2001://位置上限
					{
						p_max = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
						bSaveDataFlag = true;
					}
					break;
					case 0x2002://转速下限
					{
						w_min = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
						bSaveDataFlag = true;
					}
					break;
					case 0x2003://转速上限
					{
						w_max = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
						bSaveDataFlag = true;
					}
					break;
					case 0x2004://扭矩下限
					{
						iq_min = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
						bSaveDataFlag = true;
					}
					break;
					case 0x2005://扭矩上限
					{
						iq_max = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
						bSaveDataFlag = true;
					}
					break;
					case 0x2006://扭矩刚度
					{
						KP_MIN = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
						bSaveDataFlag = true;
					}
					break;
					case 0x2007:
					{
						KP_MAX = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
						bSaveDataFlag = true;
					}
					break;
					case 0x2008://扭矩阻尼系数
					{
						KD_MIN = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
						bSaveDataFlag = true;
					}
					break;
					case 0x2009:
					{
						KD_MAX = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
						bSaveDataFlag = true;
					}
					break;
					case 0x200A://单位：HZ，默认1000HZ  所有模式下的指令下发周期 （驱动器IP模式下备用，可忽略）
					{
						//SyncCycle = (float)((FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
					}
					break;
					case 0x200B://驱动器位置环KP（调试驱动器备用）
					{
						//Position_Kp = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
					}
					break;
					case 0x200C://驱动器速度环KP（调试驱动器备用）
					{
						//Velocity_Kp = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
					}
					break;
					case 0x200D://驱动器速度环Ki（调试驱动器备用）
					{
						//Velocity_Ki = (float)((FDCAN1_RX_DATA[7]<<24) + (FDCAN1_RX_DATA[6]<<16) + (FDCAN1_RX_DATA[5]<<8) + FDCAN1_RX_DATA[4]);
					}
					break;
//自定义指令
					case 0x2F00://can节点地址设置
					{
						FDCAN_ID = FDCAN1_RX_DATA[4];
						bSaveDataFlag = true;
					}
					break;
					case 0x2F01://电机编码设置(电机种类)
					{
						//MotorID = FDCAN1_RX_DATA[4];
					}
					break;
					case 0x2F02://编码器零位学习（0-无操作，1-回零校准）
					{
						if(FDCAN1_RX_DATA[4] == 1)
						{
							FSMstate = CALIBRATION_MODE;
							p_encoder_g->cali_start = 1;
							state_change = 1;
						}
					}
					break;
					case 0x2F03://位置模式（0-CSP模式，1-PP测试模式）
					{
						if(FDCAN1_RX_DATA[4] == 1)
						{
							p_motor_g->controlMode = FOC_POSITION_LOOP_PP;
							bPos_PP_Flag = true;
						}
						else
						{
							p_motor_g->controlMode = FOC_POSITION_LOOP;
							bPos_PP_Flag = false;
						}
					}
					break;
					case 0x2F04://软件版本号
					{
						//读取指令，返回版本号即可
					}
					break;
					case 0x2F05://测试命令（0-无操作，1-电流环带宽测试，2-测试状态上传命令）
					{
						if(FDCAN1_RX_DATA[4] == 1)
							;//
						else if(FDCAN1_RX_DATA[4] == 2)
							bDynamMode = true;//测功机测试模式，1ms周期上传扭矩速度位置，电机/MOS温度，状态信息
					}
					break;
					default:
					break;
				}
				if(bSaveDataFlag == true)
					flash_write_pending = 1;
				
				FDCAN1_TxData[0] = 0x60;
				FDCAN1_TxData[1] = FDCAN1_RX_DATA[1];
				FDCAN1_TxData[2] = FDCAN1_RX_DATA[2];
				FDCAN1_TxData[3] = FDCAN1_RX_DATA[3];
				FDCAN1_TxData[4] = FDCAN1_RX_DATA[4];
				FDCAN1_TxData[5] = FDCAN1_RX_DATA[5];
				FDCAN1_TxData[6] = FDCAN1_RX_DATA[6];
				FDCAN1_TxData[7] = FDCAN1_RX_DATA[7];
				//if(bDynamMode == false)
					CAN_SendMessage(FDCAN_ID+0x580,FDCAN1_TxData,8);
			}
			break;
			
			default:
			break;
		}
	}

	switch(Identifier)
	{
		case 0x80:{//反馈状态(广播)
			//CAN_SendMessage(FDCAN_ID+0x100,FDCAN1_TxData,8);
			pack_reply(FOC_CAN_TxData,p_encoder2_g->pos_abs,p_encoder2_g->mech_vel,p_motor_g->Q_axis_current_filt*KT_OUT,p_motor_g->Err1,p_motor_g->Err2,p_motor_g->Warning);
			//if(bDynamMode == false)
				CAN_SendMessage(0x100+FDCAN_ID,FOC_CAN_TxData,12);
		}
		break;
		case 0x200:{//速度模式
			unpack_speed_cmd(FDCAN1_RX_DATA);
			pack_reply(FOC_CAN_TxData,p_encoder2_g->pos_abs,p_encoder2_g->mech_vel,p_motor_g->Q_axis_current_filt*KT_OUT,p_motor_g->Err1,p_motor_g->Err2,p_motor_g->Warning);
			//if(bDynamMode == false)
				CAN_SendMessage(0x100+FDCAN_ID,FOC_CAN_TxData,12);
		}
		break;
		case 0x300:{//扭矩模式
			unpack_torque_cmd(FDCAN1_RX_DATA);
			pack_reply(FOC_CAN_TxData,p_encoder2_g->pos_abs,p_encoder2_g->mech_vel,p_motor_g->Q_axis_current_filt*KT_OUT,p_motor_g->Err1,p_motor_g->Err2,p_motor_g->Warning);
			//if(bDynamMode == false)
				CAN_SendMessage(0x100+FDCAN_ID,FOC_CAN_TxData,12);
		}
		break;
		case 0x400:{//位置模式
			unpack_position_cmd(FDCAN1_RX_DATA);
			pack_reply(FOC_CAN_TxData,p_encoder2_g->pos_abs,p_encoder2_g->mech_vel,p_motor_g->Q_axis_current_filt*KT_OUT,p_motor_g->Err1,p_motor_g->Err2,p_motor_g->Warning);
			//if(bDynamMode == false)
				CAN_SendMessage(0x100+FDCAN_ID,FOC_CAN_TxData,12);
		}
		break;
		case 0x500:{//MIT模式
			unpack_MIT_cmd(FDCAN1_RX_DATA);
			pack_reply(FOC_CAN_TxData,p_encoder2_g->pos_abs,p_encoder2_g->mech_vel,p_motor_g->Q_axis_current_filt*KT_OUT,p_motor_g->Err1,p_motor_g->Err2,p_motor_g->Warning);
			//if(bDynamMode == false)
				CAN_SendMessage(0x100+FDCAN_ID,FOC_CAN_TxData,12);
		}
		break;
		default:
		break;
	}

}

/// I. CAN Reply Packet Structure ///
/// 5 bits error message
/// 16 bits position, between -4*pi and 4*pi
/// 12 bits velocity, between -30 and + 30 rad/s
/// 12 bits current, between -40 and 40
/// 8 bits motor temperature
/// 8 bits MOS temperature
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
void CAN_pack_reply1(uint8_t *tData, uint8_t err, float p, float v, float i, float T1, float T2)
{
	uint16_t p_int = float_to_uint(p, POS_MIN, POS_MAX, 16);
	uint16_t v_int = float_to_uint(v, SPD_MIN, SPD_MAX, 12);
	uint16_t i_int = float_to_uint(i, I_MIN, I_MAX, 12);
	uint8_t T_motor_int = (uint8_t)(T1*2.0f+50.0f);
	uint8_t T_MOS_int = (uint8_t)(T2*2.0f+50.0f);

	tData[0] = 1<<5|err;
	tData[1] = p_int>>8;
	tData[2] = p_int;
	tData[3] = v_int>>4;
	tData[4] = v_int<<8|i_int>>8;
	tData[5] = i_int;
	tData[6] = T_motor_int;
	tData[7] = T_MOS_int;
}

/// II. CAN Reply Packet Structure ///
/// 5 bits error message
/// 32 bits position(float32)
/// 16 bits current(int16),1:100
/// 8 bits motor temperature
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
void CAN_pack_reply2(uint8_t *tData, uint8_t err, float p, float i, float T1)
{
	rv_type_convert.to_float = p;
	int16_t i_int = (int16_t)(round(i*100.0f));
	uint8_t T_motor_int = (uint8_t)(T1*2.0f+50.0f);
	tData[0] = 2<<5|err;
	tData[1] = rv_type_convert.buf[3];
	tData[2] = rv_type_convert.buf[2];
	tData[3] = rv_type_convert.buf[1];
	tData[4] = rv_type_convert.buf[0];
	tData[5] = i_int>>8;
	tData[6] = i_int;
	tData[7] = T_motor_int;
}
/// Ⅲ. CAN Reply Packet Structure ///
/// 5 bits error message
/// 32 bits velocity(float32)
/// 16 bits current(int16),1:100
/// 8 bits motor temperature
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
void CAN_pack_reply3(uint8_t *tData, uint8_t err, float v, float i, float T1)
{
	rv_type_convert.to_float = v*60.0f/2.0f/PI;
	int16_t i_int = (int16_t)(round(i*100.0f));
	uint8_t T_motor_int = (uint8_t)(T1*2.0f+50.0f);
	tData[0] = 3<<5|err;
	tData[1] = rv_type_convert.buf[3];
	tData[2] = rv_type_convert.buf[2];
	tData[3] = rv_type_convert.buf[1];
	tData[4] = rv_type_convert.buf[0];
	tData[5] = i_int>>8;
	tData[6] = i_int;
	tData[7] = T_motor_int;
}
/// Ⅳ. CAN Reply Packet Structure ///
/// 5 bits error message
/// 8 bits configuration code
/// 8 bits configuration status
/// CAN Packet is 3 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
void CAN_pack_reply4(uint8_t *tData, uint8_t err, uint8_t code, uint8_t status)
{
	tData[0] = 4<<5 | err;
	tData[1] = code;
	tData[2] = status;
}
/// Ⅴ. CAN Reply Packet Structure ///
/// 5 bits error message
/// 32 bits position(float32)
/// 16 bits current(int16),1:100
/// 8 bits motor temperature
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
void CAN_pack_reply5(uint8_t *tData, uint8_t err, uint8_t code, float data)
{
	tData[0] = 5<<5|err;
	tData[1] = code;
	rv_type_convert.to_float = data;
	tData[2] = rv_type_convert.buf[3];
	tData[3] = rv_type_convert.buf[2];
	tData[4] = rv_type_convert.buf[1];
	tData[5] = rv_type_convert.buf[0];
}

/// CAN Auto Reply Packet Structure ///
/// 16 bits position(int16),1:100
/// 16 bits velocity(int16),1:10
/// 16 bits current(int16),1:100
/// 8 bits motor temperature
/// 8 bits error message
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
void CAN_pack_autoReply(uint8_t *tData, float p, float v, float i, float T, uint8_t err)
{
	int16_t pos = (int16_t)(p*100.0f*180.0f/PI);
	int16_t vel = (int16_t)(v*10.0f*60.0f/2.0f/PI);
	int16_t cur = (int16_t)(round(i*100.0f));
	tData[0] = pos>>8;
	tData[1] = pos;
	tData[2] = vel>>8;
	tData[3] = vel;
	tData[4] = cur>>8;
	tData[5] = cur;
	tData[6] = (uint8_t)(T*2.0f+50.0f);
	tData[7] = err;
}