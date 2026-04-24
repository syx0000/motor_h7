/*************interact.h*********************/
 
#ifndef _INTERACT_H
#define _INTERACT_H
#include "main.h"

/*PD控制*/
//#define P_MIN   -500.0f
//#define P_MAX   500.0f
//#define V_MIN   -10.0f
//#define V_MAX   10.0f
//#define KP_MIN  0.0f
//#define KP_MAX  5.0f
//#define KD_MIN  0.0f
//#define KD_MAX  5.0f
//#define T_MIN   -5.0f
//#define T_MAX   5.0f


extern uint32_t CAN_ID;
extern uint16_t CAN_MASTER;
extern float I_BW;

extern float Motor_Iq; 
extern float Motor_W; 
extern float Motor_P;

extern float FOC_velAccDec;

extern volatile uint16_t state_change;

void enter_menu_state(void);
void enter_setup_state(void);
 
#endif

