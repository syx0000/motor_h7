#ifndef __CAN_RV_H
#define __CAN_RV_H
#include "main.h"
#include "FOC.h"

extern bool bDynamMode;
extern bool bTargetPosFinish;

extern char CommMode;
extern uint32_t CAN_timeout;
extern uint32_t CAN_TIMEOUT;
extern uint8_t FOC_CAN_TxData[16];

extern float KP_MIN;
extern float KP_MAX;
extern float KD_MIN;
extern float KD_MAX;

/*通信模式*/
#define CommAutoFeedback	1
#define CommResponseMode	2

#define CanID_Upload_Current	0x700
#define CanID_Upload	0x7FE

void CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t len);
void CAN_Config(void);

void pack_reply(uint8_t *tData,float pos,	float vel, float torque, uint16_t err1, uint8_t err2, uint8_t warning);//反馈帧
void Pack_ActiveReport_Current(uint8_t *tData ,float torque);
void Pack_ActiveReport(uint8_t *tData,float pos, float vel, float torque, uint16_t err1, uint8_t err2, uint8_t warning);
//void PD_pack_reply(uint8_t *tData, float p, float v, float t);
//void unpack_cmd(uint8_t *CAN_RxData, ControllerStruct * controller);
void unpack_speed_cmd(uint8_t CAN_RxData[]);
void unpack_torque_cmd(uint8_t CAN_RxData[]);
void unpack_position_cmd(uint8_t CAN_RxData[]);
void unpack_MIT_cmd(uint8_t CAN_RxData[]);

void CAN_MsgProcess(uint32_t Identifier, uint8_t *FDCANRxData);

void CAN_pack_reply1(uint8_t *tData, uint8_t err, float p, float v, float i, float T1, float T2);
void CAN_pack_reply2(uint8_t *tData, uint8_t err, float p, float i, float T1);
void CAN_pack_reply3(uint8_t *tData, uint8_t err, float v, float i, float T1);
void CAN_pack_reply4(uint8_t *tData, uint8_t err, uint8_t code, uint8_t status);
void CAN_pack_reply5(uint8_t *tData, uint8_t err, uint8_t code, float data);
void CAN_pack_autoReply(uint8_t *tData, float p, float v, float i, float T, uint8_t err);
//#include "delay.h"

//#define param_get_pos	0x01
//#define param_get_spd	0x02
//#define param_get_cur	0x03
//#define param_get_pwr	0x04
//#define param_get_acc	0x05
//#define param_get_lkgKP	0x06
//#define param_get_spdKI	0x07
//#define param_get_fdbKP	0x08
//#define param_get_fdbKD	0x09

//#define comm_ack	0x00
//#define comm_auto	0x01

//typedef struct 
//{
//	uint16_t motor_id;
//	uint8_t INS_code;		//instruction code.
//	uint8_t motor_fbd;	//motor CAN communication feedback.
//}MotorCommFbd;

//typedef struct 
//{
//	uint16_t angle_actual_int;
//	uint16_t angle_desired_int;
//	int16_t speed_actual_int;
//	int16_t speed_desired_int;
//	int16_t current_actual_int;
//	int16_t current_desired_int;
//	float 	speed_actual_rad;
//	float 	speed_desired_rad;
//	float 	angle_actual_rad;	
//	float   angle_desired_rad;
//	uint16_t	motor_id;
//	uint8_t 	temperature;
//	uint8_t		error;
//	float     angle_actual_float;
//	float 		speed_actual_float;
//	float 		current_actual_float;
//	float     angle_desired_float;
//	float 		speed_desired_float;
//	float 		current_desired_float;
//	float			power;
//	uint16_t	acceleration;
//	uint16_t	linkage_KP;
//	uint16_t 	speed_KI;
//	uint16_t	feedback_KP;
//	uint16_t	feedback_KD;
//}OD_Motor_Msg;

//extern OD_Motor_Msg rv_motor_msg[8];
//extern uint16_t motor_id_check;



#endif
