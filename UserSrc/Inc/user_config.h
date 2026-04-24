#ifndef user_config_H
#define user_config_H
//#define JM57_DP18_DE_C//定义模组型号
//#define JM65_DP28_DE_C//定义模组型号
//#define FIVE_2808//定义模组型号

//#if defined(JM65_DP28_DE_C)
//	#define GR 28.0f//模组减速比
//	#define V_CAL 0.6f;
//	/*FOC控制限幅*/
//	#define iq_min   -10.0f
//	#define iq_max   10.0f
//	#define w_min   -209.4395f
//	#define w_max   209.4395f
//	#define p_min   -628.3185f
//	#define p_max   628.3185f
//#elif defined(JM57_DP18_DE_C)
//	#define GR 18.0f//模组减速比
//	#define V_CAL 4.8f;
//	/*FOC控制限幅*/
//	#define iq_min   -10.0f
//	#define iq_max   10.0f
//	#define w_min   -209.4395f
//	#define w_max   209.4395f
//	#define p_min   -628.3185f
//	#define p_max   628.3185f
//#elif defined(FIVE_2808)
//	#define GR 18.0f//模组减速比
//	#define V_CAL 0.6f;
//	/*FOC控制限幅*/
//	#define iq_min   -10.0f
//	#define iq_max   10.0f
//	#define w_min   -5000.0f
//	#define w_max   5000.0f
//	#define p_min   -628.3185f
//	#define p_max   628.3185f
//#else
//    #error "未定义产品型号！"
//#endif

#define GR 25.0f//模组减速比
#define V_CAL 1.5f;
/*FOC控制限幅*/
//#define iq_min   -60.2f
//#define iq_max   60.2f
//#define w_min   -500.0f
//#define w_max   500.0f
//#define p_min   -628.3185f
//#define p_max   628.3185f

extern float I_SWOver;
extern float Position_P;
extern float Position_I;
extern float Velocity_P;
extern float Velocity_I;
extern float Current_P;
extern float Current_I;

extern float iq_min;
extern float iq_max;
extern float w_min;
extern float w_max;
extern float p_min;
extern float p_max;

#endif
