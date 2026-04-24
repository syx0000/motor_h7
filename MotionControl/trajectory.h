/**
  ******************************************************************************
  * @file           : trajectory.h
  * @author         : ZQM
  * @brief          : brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

/* ------------------------------ Includes ------------------------------ */
#include "main.h"
/* ------------------------------ Defines ------------------------------ */
#define NumberOfInterTimes 2001   //最大插值数目
/* ------------------------------ Enum Typedef ------------------------------ */
extern unsigned char traj_complete;        //规划完成标志
extern uint8_t trajectory_tim_count;       // tim1中断次数计数
extern unsigned char trace_task_complete;  //本次规划跟踪到位完成标志
/* ------------------------------ Struct Typedef ------------------------------ */
typedef struct _Trajectory_t
{
	 float vel_max;
	 float acc;
	 float dec;
	 float acc_max;//最大加速度
	 float dec_max;//最大减速度
	 float pos1;  //起始位置
	 float pos2;  //终点位置
	
	 uint16_t p_data_max_num;  //规划次数
	 float t1 ;  //加速时间
	 float t2 ;  //匀速时间
	 float t3 ;  //减速时间
	
	 float pos_demand[NumberOfInterTimes] ;
	 float vel_demand[NumberOfInterTimes] ;
	 float acc_demand[NumberOfInterTimes] ;
}Trajectory_t;


/* ------------------------------ Manager Typedef ------------------------------ */
typedef struct TrajectoryManager
{
/**
  * @brief  Init
  * @note   note.
  * @param  param brief
  * @retval None
  */
  void (*Get_Trape_Para)(Trajectory_t *trajectory);

/**
  * @brief  Sample
  * @note   note.
  * @param  param brief
  * @retval None
  */
  void (*Get_Trape_Plan)(Trajectory_t *trajectory);
	

}TrajectoryManager_typedef;
/* ------------------------------ Manager Extern ------------------------------ */
extern const TrajectoryManager_typedef Trajectory;
extern uint16_t p_data_soft_num ;  
#endif
