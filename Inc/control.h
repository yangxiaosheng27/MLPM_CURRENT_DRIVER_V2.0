/**
  ******************************************************************************
  * File Name          : control.h
  * Description        : This file provides code for the current control.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __control_H
#define __control_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define DOUBLE_POLE						1		//双极性PWM
#define SINGLE_POLE						2		//单极性PWM
	 
#define PWM_POLE  SINGLE_POLE			//使用单极性PWM!!!

#define PWM_REGISTER_A_L (TIM8->CCR3)
#define PWM_REGISTER_A_R (TIM8->CCR4)
#define PWM_REGISTER_B_L (TIM8->CCR1)
#define PWM_REGISTER_B_R (TIM8->CCR2)
#define PWM_REGISTER_C_L (TIM1->CCR3)
#define PWM_REGISTER_C_R (TIM1->CCR4)
#define PWM_REGISTER_D_L (TIM1->CCR1)
#define PWM_REGISTER_D_R (TIM1->CCR2)

#define 	Initial_Kp	 			0.225f
#define 	Initial_Ki	 			0.0275f
#define		Initial_Kd				0.0f
//PID控制参数结构体
typedef struct
{
	f32 Kp;									//P参数
	f32 Ki;									//I参数
	f32 Kd;									//D参数
	int32_t Err_Now;				//当前时刻误差
	int32_t Err_Last;				//上一时刻误差
	int32_t dErr;						//相邻时刻误差的差分值
	int32_t Sum_Err;				//累积误差
	int32_t Max_SumErr;			//累积误差限幅
	int32_t Err_Threshold;	//判断使能积分控制的误差临界值
	int32_t Output;					//控制量输出
	int32_t Max_Output;			//控制量输出限幅
} PID_Struct;
	 
// 电流控制参数结构体
typedef struct
{
	int32_t Expect_Value;									//电流期望值(mA)
	int32_t Feedback_Value;								//电流反馈值(mA)
	int32_t Sample_Value;									//ADC值经计算后得到的原始采样值(mA)，未滤波处理
	uint16_t Bias_Value;									//电流偏移量:电流为零时的ADC值
	int32_t PWM_Duty;											//PWM占空比，可正可负
	int32_t Max_PWM_Duty;									//PWM占空比上限
	__IO uint32_t *PWM_Register_Left;			//控制左半桥上桥臂PWM占空比的寄存器
	__IO uint32_t *PWM_Register_Right;		//控制右半桥上桥臂PWM占空比的寄存器
	PID_Struct PID;												//PID控制参数

} Current_Struct;

extern PID_Struct PID_A;	 
extern PID_Struct PID_B;	 
extern PID_Struct PID_C;
extern PID_Struct PID_D;	 

extern Current_Struct Current_A;
extern Current_Struct Current_B;
extern Current_Struct Current_C;
extern Current_Struct Current_D;

void Current_Controller_Init(void);						  //控制参数初始化

void Current_Loop_Control(uint32_t Board_State);//电流环控制

void PWM_Duty_Updata(Current_Struct *Current);//更新PWM占空比相关寄存器

void Controller_Parameter_Init(Current_Struct *Current, f32 Kp, f32 Ki, f32 Kd, 					\
																int32_t Max_PID_SumErr, int32_t Max_PID_Output, 	\
																int32_t Err_Threshold, int32_t Max_PWM_Duty,		\
																__IO uint32_t *PWM_Register_Left,					\
																__IO uint32_t *PWM_Register_Right);

int32_t PID_Calculate(PID_Struct *PID, int32_t Expect_Value, int32_t Feedback_Value);//计算PID控制量

void Coilx_PWM_Duty_Update(Current_Struct *Current);
void Coilx_Stop(Current_Struct *Current);
void AllCoils_Stop(void);
static int32_t PID_Calculate(PID_Struct *PID, int32_t Expect_Value, int32_t Feedback_Value);

#ifdef __cplusplus
}
#endif

#endif /*__control_H */


/**** ***END OF FILE  ****/
