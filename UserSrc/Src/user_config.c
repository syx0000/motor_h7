#include "main.h"
#include "user_config.h"

float I_SWOver = 58.0f;//单位：A
/*PI参数初始值*/
/*90空载位置闭环PP模式*/
//float Position_P = 300.0f;
//float Position_I = 0.00000f;
//float Velocity_P = 0.1f;
//float Velocity_I = 0.00005f;
//float Current_P = 0.11854f;
//float Current_I = 0.001625f;

//float Position_P = 300.0f;
//float Position_I = 0.0000001f;
//float Velocity_P = 0.5f;
//float Velocity_I = 0.00005f;
//float Current_P = 0.21536666f;
//float Current_I = 0.0158666667;
/*90空载位置闭环阶跃模式-位置环输出限幅2000rpm*/
//float Position_P = 50.0f;
//float Position_I = 0.0000001f;
//float Velocity_P = 0.5f;
//float Velocity_I = 0.00005f;
//float Current_P = 0.21536666f;
//float Current_I = 0.0158666667;
/*转矩控制精度*/
//float Position_P = 50.0f;
//float Position_I = 0.0000001f;
//float Velocity_P = 0.5f;
//float Velocity_I = 0.00005f;
////float Current_P = 0.21536666f;
////float Current_I = 0.0158666667;
//float Current_P = 0.43073332f;
//float Current_I = 0.0317333334;//4k期望带宽
/*90空载速度/电流闭环*/
//float Position_P = 300.0f;
//float Position_I = 0.0000001f;
//float Velocity_P = 1.5f;
//float Velocity_I = 0.00005f;
//float Current_P = 0.11854f;
//float Current_I = 0.001625f;
/*90带载速度/电流闭环-完成额定工况运行*/
//float Position_P = 300.0f;
//float Position_I = 0.0000001f;
//float Velocity_P = 1.5f;
//float Velocity_I = 0.00005f;
//float Current_P = 0.10768333f;
//float Current_I = 0.001625f;

//230
float Position_P = 300.0f;
float Position_I = 0.0000001f;
float Velocity_P = 1.0f;
float Velocity_I = 0.0001f;
float Current_P = 0.10768333f;
float Current_I = 0.0005;//0.001625f;//
//float Current_P = 0.20768333f;
//float Current_I = 0.01f;

//230想突破到额定的尝试
//float Position_P = 300.0f;
//float Position_I = 0.0000001f;
//float Velocity_P = 0.7f;
//float Velocity_I = 0.005f;
//float Current_P = 0.30768333f;//改为0.50768333f后TVS管烧毁
//float Current_I = 0.005625f;

//float Position_P = 0.0f;
//float Position_I = 0.000000f;
//float Velocity_P = 0.7f;
//float Velocity_I = 0.005f;
//float Current_P = 0.20768333f;
//float Current_I = 0.01f;

/*90带载位置闭环-完成额定工况运行*/
//float Position_P = 300.0f;
//float Position_I = 0.0000001f;
////float Velocity_P = 1.5f;
////float Velocity_I = 0.00005f;
//float Velocity_P = 1.0f;//＞1.5给额定转速会震荡；1.5位置闭环噪声较大（转速波动大），可调小，1.0会好很多
//float Velocity_I = 0.001f;//参数0.00005f时带载工况下，开始几个点的位置跟踪误差大，速度曲线跟踪差，可增大积分系数
//float Current_P = 0.10768333f;
//float Current_I = 0.001625f;
/*90空载电流扫频测试*/
//float Position_P = 0.0f;
//float Position_I = 0.00000f;
//float Velocity_P = 0.0f;
//float Velocity_I = 0.00000f;
//float Current_P = 0.10768333f;
//float Current_I = 0.0079333333;//1k期望带宽

//float Current_P = 0.21536666f;
//float Current_I = 0.0158666667;//2k期望带宽

//float Current_P = 0.32304999f;
//float Current_I = 0.0237999999f;//3k期望带宽

//float Current_P = 0.32304999f;
//float Current_I = 0.01f;//3k期望带宽

//float Current_P = 0.43073332f;
//float Current_I = 0.0317333334;//4k期望带宽

//float Current_P = 0.53841665f;
//float Current_I = 0.0396666665;//5k期望带宽

//float Current_P = 0.64609998f;
//float Current_I = 0.0476000001;//6k期望带宽√

//float Current_P = 0.86146664f;
//float Current_I = 0.0634666668;//8k期望带宽

//参数总结：
/*转矩控制精度参数*/
//float Current_P = 0.20768333f;
//float Current_I = 0.01f;
/*电流闭环/MIT模式*/
//float Position_P = 0.0f;
//float Position_I = 0.00000f;
//float Velocity_P = 0.0f;
//float Velocity_I = 0.00000f;
//float Current_P = 0.10768333f;
//float Current_I = 0.0079333333;//1k期望带宽
////float Current_P = 0.21536666f;
////float Current_I = 0.0158666667;//2k期望带宽

/*速度闭环*/
//float Position_P = 0.0f;
//float Position_I = 0.00000f;
//float Velocity_P = 1.5f;
//float Velocity_I = 0.00005f;
//float Current_P = 0.10768333f;
//float Current_I = 0.0079333333;//1k期望带宽

/*位置闭环*/
//float Position_P = 300.0f;
//float Position_I = 0.0000001f;
//float Velocity_P = 1.0f;
//float Velocity_I = 0.001f;
//float Current_P = 0.10768333f;
//float Current_I = 0.001625f;



//#if defined(JM65_DP28_DE_C)
//	float I_SWOver = 15.0f;//单位：A
//	/*PI参数初始值*/
//	float Position_P = 3.0f;
//	float Position_I = 0.00002f;
//	float Velocity_P = 0.5f;
//	float Velocity_I = 0.00005f;
//	float Current_P = 0.21854f;
//	float Current_I = 0.001625f;

//#elif defined(JM57_DP18_DE_C)
//	float I_SWOver = 10.0f;//单位：A
//	/*PI参数初始值*/
//	float Position_P = 3.0f;
//	float Position_I = 0.00002f;
//	float Velocity_P = 0.3f;
//	float Velocity_I = 0.00005f;
//	float Current_P = 0.21854f;
//	float Current_I = 0.001625f;
////	float Velocity_P = 0.15f;
////	float Velocity_I = 0.00001f;
////	float Current_P = 0.05f;
////	float Current_I = 0.001f;

//#elif defined(FIVE_2808)
//	float I_SWOver = 15.0f;//单位：A
//	/*PI参数初始值*/
//	float Position_P = 3.0f;
//	float Position_I = 0.00002f;
//	float Velocity_P = 0.3f;
//	float Velocity_I = 0.00005f;
//	float Current_P = 0.21854f;
//	float Current_I = 0.001625f;
////	float Velocity_P = 0.15f;
////	float Velocity_I = 0.00001f;
////	float Current_P = 0.05f;
////	float Current_I = 0.001f;
//#else
//    #error "未定义产品型号！"
//#endif

//转矩/速度/位置 限幅值
float iq_min = -500.0f;
float iq_max = 500.0f;
float w_min = -20.0f;
float w_max = 20.0f;
float p_min = -7;
float p_max = 7;

