#include "interact.h"
#include "FOC.h"
#include "main.h"
#define VERSION_NUM "Servo-V1000"
uint32_t CAN_ID = 0x01;
uint16_t CAN_MASTER = 0x01;
float I_BW = 1000.0f;//单位：Hz
float Motor_Iq = 0;//单位：A 电流指令对外接口，can通信或者串口通信的目标电流都是先幅值给Motor_Iq
float Motor_W = 0;//单位：rad/s(电机端)
float Motor_P = 0;//单位：rad(电机端) 位置指令对外接口，can通信或者串口通信的目标位置都是先幅值给Motor_P

float FOC_velAccDec = 200;//速度规划 加/减速度 单位rad/s/s

volatile uint16_t state_change = 0;

void EnterMenuState(void)
{
	printf("Firmware Version: %s\r", VERSION_NUM);//打印软件版本
	DisablePWM();
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
	p_velocity_loop_g->output = 0;
	p_velocity_loop_g->target = 0;
	p_velocity_loop_g->P = 0;
	p_velocity_loop_g->I = 0;
	
    printf("\n\r");
    printf(" Commands:\n\r");
    DelayUs(10);
    printf(" m - Motor Mode\n\r");
    DelayUs(10);
    printf(" c - Calibrate Encoder\n\r");
    DelayUs(10);
    printf(" s - Setup\n\r");
    DelayUs(10);
	printf(" h - Homing\n\r");
    DelayUs(10);
    printf(" e - Display Encoder\n\r");
    DelayUs(10);
    printf(" z - Set Zero Position\n\r");
    DelayUs(10);
    printf(" esc - Exit to Menu\n\r");
    DelayUs(10);
    state_change = 0;
}

void EnterSetupState(void)
{
	printf("\n\r Configuration Options \n\r");
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %-2s\n\r\n\r", "prefix", "parameter", "min", "max", "current value");
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %-5i\n\r", "c", "Control Mode", "0", "4", p_motor_g->controlMode);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %.1f\n\r", "b", "Current Bandwidth (Hz)", "10", "2000", I_BW);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %-5i\n\r", "i", "CAN ID", "0", "127", CAN_ID);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %-5i\n\r", "m", "CAN Master ID", "0", "127", CAN_MASTER);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %.1f\n\r", "o", "Current Software Over(A)", "0.0", "50.0", I_SWOver);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %.7f\n\r", "P", "Velocity Loop P_Coefficient", "0.0", "1.0", p_velocity_loop_g->kp);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %.7f\n\r", "I", "Velocity Loop I_Coefficient", "0.0", "1.0", p_velocity_loop_g->ki);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %.7f\n\r", "M", "Position Loop P_Coefficient", "0.0", "100.0", p_position_loop_g->kp);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %.7f\n\r", "N", "Position Loop I_Coefficient", "0.0", "1.0", p_position_loop_g->ki);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %.7f\n\r", "Q", "Current Loop P_Coefficient", "0.0", "1.0", controller.k_d);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %.7f\n\r", "W", "Current Loop I_Coefficient", "0.0", "1.0", controller.ki_d);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %.5f\n\r", "A", "Velocity Loop Acc/Dec", "0.0", "5000.0", FOC_velAccDec);
	DelayUs(10);
	printf(" %-6s %-31s %-5s %-6s %d\n\r", "t", "CAN Timeout (cycles)(0 = none)", "0", "100000", CAN_TIMEOUT);
	DelayUs(10);
//	printf(" %-6s %-31s %-5s %-6s %.5f\n\r", "V", "W_interval(rad/s)", "0.0", "5000.0", W_interval);
//	DelayUs(10);
//	printf(" %-6s %-31s %-5s %-6s %d\n\r", "T", "T_interval(ms)", "0", "100000", T_interval);
//	DelayUs(10);
	printf("\n\r To reset HW OverCurrent, type 'r''0''ENTER'");
	printf("\n\r To change a value, type 'prefix''value''ENTER'\n\r i.e. 'b1000''ENTER'\n\r\n\r");
	DelayUs(10);
	state_change = 0;
}
