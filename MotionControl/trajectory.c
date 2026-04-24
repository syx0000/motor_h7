/**
 ******************************************************************************
 * @file           : trajectory.c
 * @author         : ZQM
 * @brief          : brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
/* ------------------------------ Includes ------------------------------ */
#include "trajectory.h"
#include <FOC.h>
/* ------------------------------ Defines ------------------------------ */

/* ------------------------------ Private Function Declarations ------------------------------ */

/* ------------------------------ Public Function Declarations ------------------------------ */
static void Get_Trape_Para(Trajectory_t *trajectory);
static void Get_Trape_Plan(Trajectory_t *trajectory);


/* ------------------------------ Variable Declarations ------------------------------ */
Trajectory_t trajectory_g;
Trajectory_t *p_trajectory_g = &trajectory_g;
uint16_t p_data_soft_num = 0;          //已实现步数计数
float p_period = PeriodPWM * 20.0f;    // 规划控制周期 2ms 位置环1ms执行一次
uint8_t trajectory_tim_count = 0;      // tim1中断次数计数
uint16_t Max_num = 2000;			// 2000 *0.002 = 4s 最大规划时间为4s，如果无法全部执行则仅执行2000次
unsigned char traj_active = 0x00;  //规划有效标志
unsigned char traj_complete = 0x00;  //规划完成标志
unsigned char trace_task_complete = 0x00;  //本次规划跟踪到位完成标志


/* ------------------------------ Private ------------------------------ */
/**
  * @brief  brief.
  * @note   note.
  * @param  param brief
  * @retval None
  */						
static void Get_Trape_Para(Trajectory_t *trajectory)//根据给定参数（pos1、pos2、acc、dec、vel_max）计算加减速时间和可以到达的最大速度
{
	//加速度、减速度、最大速度全部给正值，规划时再判断
    float ta = trajectory->vel_max/trajectory->acc;//加速时间
    float td = trajectory->vel_max/trajectory->dec;//减速时间
    float sa = 0.5f * trajectory->acc*ta*ta;//1/2*a*t_2：加速路程
    float sd = 0.5f * trajectory->dec*td*td;//1/2*a*t_2：减速路程
	  float sq = trajectory->pos2 - trajectory->pos1;
	  traj_complete = 0x00;  //规划完成标志清零
		
		if(sq < 0)
		{
			sq = -sq;
		}
	//如果最大规划时间全程按照最大加速度(500rad/s^2)和最大速度运动也无法到达指定位置，则放弃规划
		if(sq > trajectory->vel_max * (p_period * Max_num - ta - td) + sa +sd)//396.9474432 按照加速度和减速的的参数运行，最长路程不是vel_max * p_period * Max_num，而是vel_max * (p_period * Max_num - ta - td) + sa +sd
		{
			trajectory->p_data_max_num = 0;
			traj_active = 0x00;
//			HAL_Delay(1000);阶跃太大会卡死在这里
			printf("\r\nPlan failed \r\n");
			return;
		}
    if(sa+sd > sq)//21.9325568 加减速时间内的路程已经大于要走的路程，对vel_max重新计算赋值
    {
			trajectory->vel_max = sqrt(2.0f * sq / (1.0f / trajectory->acc + 1.0f / trajectory->dec));
			trajectory->t1 = trajectory->vel_max/trajectory->acc;
			trajectory->t2 = 0;
			trajectory->t3 = trajectory->vel_max/trajectory->dec;
			traj_active = 0x01;
			printf("\r\nSpeed triangle \r\n");
    }
    else
    {
			trajectory->t1 = ta;
			trajectory->t2 = (sq-sa-sd)/trajectory->vel_max;
			trajectory->t3 = td;
			traj_active = 0x01;
    }
		float total_time = trajectory->t1 + trajectory->t2 + trajectory->t3;
		trajectory->p_data_max_num = (uint16_t) (total_time / p_period) + 1;    //根据计算出的到达时间计算规划次数 浮点数的整数部分会被保留，小数部分会被直接丢弃
	  if(trajectory->p_data_max_num > Max_num)//直接在判断sq > trajectory->vel_max * p_period * Max_num时改为与vel_max * (p_period * Max_num - ta - td) + sa +sd比较
	  {
			//规划数组超出数组最大范围，按照最大规划时间、最大加速度、最大减速度重新规划路线
			//待规划
			//超出范围后数组最后一个值直接给pos2阶跃？
				trajectory->p_data_max_num = Max_num;
	  }
		//printf(" %f \r\n",total_time);
}

static void Get_Trape_Plan(Trajectory_t *trajectory)//将加速度、速度、位置离散化存储进数组
{
		float sq = trajectory->pos2 - trajectory->pos1;	
		float acc =	trajectory->acc;
		float dec =	trajectory->dec;
		float vmax =	trajectory->vel_max;
		if(sq < 0)
		{
			acc = -acc;
			dec = -dec;
			vmax = -vmax;			 
		}
	
    float v1 = trajectory->t1*acc;
    float v2 = v1;
    float p1 = trajectory->pos1 + 0.5f*acc*trajectory->t1*trajectory->t1;
    float p2 = p1+v1*trajectory->t2;
    float temp_t = 0.0f;
	  
	if(traj_active)
		{
    for(uint16_t i=0;i <= trajectory->p_data_max_num; i++)
			{
        if ((trajectory->t1 - i*p_period) >= 0.00001f)
        {
            temp_t = i * p_period;
            trajectory->acc_demand[i] = acc;
            trajectory->vel_demand[i] = acc*temp_t;
            trajectory->pos_demand[i] = trajectory->pos1 + 0.5f * acc * temp_t * temp_t;
        }
        else if((trajectory->t1+trajectory->t2-i*p_period) >= 0.00001f)
        {
            temp_t = i*p_period - trajectory->t1;
            trajectory->acc_demand[i] = 0.0f;
            trajectory->vel_demand[i] = v1;
            trajectory->pos_demand[i] = p1+v1*temp_t;
        }
        else if((trajectory->t1+trajectory->t2+trajectory->t3-i*p_period) >= 0.00001f)
        {
            temp_t = i*p_period - trajectory->t1-trajectory->t2;
            trajectory->acc_demand[i] = -dec;
            trajectory->vel_demand[i] = v2 - dec*temp_t;
            trajectory->pos_demand[i] = p2+0.5f*(v2+trajectory->vel_demand[i])*temp_t;
        }
        else
        {
            temp_t = trajectory->t3;
            trajectory->acc_demand[i] = 0.0f;
            trajectory->vel_demand[i] = 0.0f;
            trajectory->pos_demand[i] = p2+0.5f*(v2+trajectory->vel_demand[i])*temp_t;
        }
			}
		traj_complete = 0x01; //规划完成
		}	
}

/* ------------------------------ Manager Declaration ------------------------------ */
const TrajectoryManager_typedef Trajectory =
{
  Get_Trape_Para,
  Get_Trape_Plan
};
