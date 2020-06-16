/**
  ******************************************************************************
  * File Name          : control.c
  * Description        : This file provides code for the current control.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
	
#include "control.h"

//四相电流的电流控制参数
Current_Struct Current_A;
Current_Struct Current_B;
Current_Struct Current_C;
Current_Struct Current_D;

//用于存储变比例积分参数的中间变量
f32 Temp_Kp=0;
f32 Temp_Ki=0;

//电流环控制
void Current_Loop_Control(uint32_t Board_State)
{
	if(Running==Board_State)	//板子接收到电流信号，进入Running状态
	{
		Current_A.PWM_Duty = PID_Calculate(&Current_A.PID, Current_A.Expect_Value, Current_A.Feedback_Value);
		
		Current_B.PWM_Duty = PID_Calculate(&Current_B.PID, Current_B.Expect_Value, Current_B.Feedback_Value);
				
		Current_C.PWM_Duty = PID_Calculate(&Current_C.PID, Current_C.Expect_Value, Current_C.Feedback_Value);
		
		Current_D.PWM_Duty = PID_Calculate(&Current_D.PID, Current_D.Expect_Value, Current_D.Feedback_Value);
		
		Coilx_PWM_Duty_Update(&Current_A);
		Coilx_PWM_Duty_Update(&Current_B);
		Coilx_PWM_Duty_Update(&Current_C);
		Coilx_PWM_Duty_Update(&Current_D);
	}
	else											//Standby或者其他错误状态，关闭所有线圈的电流输出,积分值清零
	{
		AllCoils_Stop();
		Current_A.Expect_Value=0U;
		Current_B.Expect_Value=0U;
		Current_C.Expect_Value=0U;
		Current_D.Expect_Value=0U;
		Current_A.PID.Sum_Err=0U;
		Current_B.PID.Sum_Err=0U;
		Current_C.PID.Sum_Err=0U;
		Current_D.PID.Sum_Err=0U;
	}		
}

//控制初始化
void Current_Controller_Init(void)
{	
	f32 Kp	= Initial_Kp;
	f32 Ki	= Initial_Ki;
	f32 Kd	= Initial_Kd;	
	int32_t Err_Threshold = 2000U;	//mA
	Controller_Parameter_Init(&Current_A, Kp, Ki, Kd, MAX_SUM_ERR, MAX_PWM_DUTY, Err_Threshold, MAX_PWM_DUTY,&PWM_REGISTER_A_L,&PWM_REGISTER_A_R);
	Controller_Parameter_Init(&Current_B, Kp, Ki, Kd, MAX_SUM_ERR, MAX_PWM_DUTY, Err_Threshold, MAX_PWM_DUTY,&PWM_REGISTER_B_L,&PWM_REGISTER_B_R);
	Controller_Parameter_Init(&Current_C, Kp, Ki, Kd, MAX_SUM_ERR, MAX_PWM_DUTY, Err_Threshold, MAX_PWM_DUTY,&PWM_REGISTER_C_L,&PWM_REGISTER_C_R);
	Controller_Parameter_Init(&Current_D, Kp, Ki, Kd, MAX_SUM_ERR, MAX_PWM_DUTY, Err_Threshold, MAX_PWM_DUTY,&PWM_REGISTER_D_L,&PWM_REGISTER_D_R);
}

//控制参数初始化，被Control_Init调用
void Controller_Parameter_Init(Current_Struct *Current, f32 Kp, f32 Ki, f32 Kd, 					\
												int32_t Max_PID_SumErr, int32_t Max_PID_Output, 	\
												int32_t Err_Threshold, int32_t Max_PWM_Duty,		\
												__IO uint32_t *PWM_Register_Left,					\
												__IO uint32_t *PWM_Register_Right)
{
	Current->Expect_Value 					= 	0.0f;
	Current->Feedback_Value 				= 	0.0f;;
	Current->Sample_Value 					= 	0.0f;;
	Current->PWM_Register_Left 		 		= 	PWM_Register_Left;
	Current->PWM_Register_Right 			= 	PWM_Register_Right;
	Current->Max_PWM_Duty					= 	Max_PWM_Duty;
	
	#if PWM_POLE == SINGLE_POLE
		Current->PWM_Duty					= 	0U;
	#elif PWM_POLE == DOUBLE_POLE
		Current->PWM_Duty					= 	Max_PWM_Pulse/2;
	#else
		Current->ErrorCode				|	= 	ERROR_PWM_POLE;
	#endif //PWM_POLE
	
	Current->PID.Kp 						= 	Kp;
	Current->PID.Ki 						= 	Ki;
	Current->PID.Kd 						= 	Kp;
	Current->PID.dErr						= 	0;
	Current->PID.Err_Last					= 	0;
	Current->PID.Err_Now					= 	0;
	Current->PID.Err_Threshold				=	Err_Threshold;
	Current->PID.Max_Output					= 	Max_PID_Output;
	Current->PID.Max_SumErr					= 	Max_PID_SumErr;
	Current->PID.Output						= 	Current->PWM_Duty;
	Current->PID.Sum_Err					= 	0;
}
static int32_t PID_Calculate(PID_Struct *PID, int32_t Expect_Value, int32_t Feedback_Value)
{
	/*
	Temp_Kp	=	PID->Kp;
	
	if(Expect_Value>0)
	Temp_Ki = PID->Ki+Expect_Value/15000.0*0.09;		//积分参数从0-15A线性增长
	else
	Temp_Ki = PID->Ki+(-Expect_Value)/15000.0*0.09;		//积分参数从0-15A线性增长	
	*/	
	PID->Err_Now = Expect_Value - Feedback_Value;
	if(PID->Err_Now > PID->Err_Threshold || PID->Err_Now < -PID->Err_Threshold)
	{
		PID->Sum_Err = 0;
	}
	else
	{
		PID->Sum_Err += PID->Err_Now;
		if(PID->Sum_Err > PID->Max_SumErr)
		{
			PID->Sum_Err = PID->Max_SumErr;
		}
		else if(PID->Sum_Err < -PID->Max_SumErr)
		{
			PID->Sum_Err = -PID->Max_SumErr;
		}
	}
	PID->dErr = PID->Err_Now - PID->Err_Last;
//	PID->Output = Temp_Kp * PID->Err_Now + Temp_Ki * PID->Sum_Err + PID->Kd * PID->dErr;
	PID->Output = PID->Kp * PID->Err_Now + PID->Ki * PID->Sum_Err + PID->Kd * PID->dErr;
	if(PID->Output > PID->Max_Output)
	{
		PID->Output = PID->Max_Output;
	}
	else if(PID->Output < -PID->Max_Output)
	{
		PID->Output = -PID->Max_Output;
	}
	PID->Err_Last = PID->Err_Now;
	
	return PID->Output;
}

//更新Coilx的PWM占空比相关寄存器
void Coilx_PWM_Duty_Update(Current_Struct *Current)
{
	if(Current->PWM_Duty >= 0)
	{
		*(Current->PWM_Register_Left) 	= 	Current->PWM_Duty;
		*(Current->PWM_Register_Right) 	= 	0U;
	}
	else
	{
		*(Current->PWM_Register_Left) 	= 	0U;
		*(Current->PWM_Register_Right) 	= 	-Current->PWM_Duty;
	}
}
/********关闭某一个线圈*********/
void Coilx_Stop(Current_Struct *Currentx)
{
	*(Currentx->PWM_Register_Left) 	= 	0U;
	*(Currentx->PWM_Register_Right) = 	0U;
}
/********关闭全部线圈*********/
void AllCoils_Stop(void)
{
	Coilx_Stop(&Current_A);
	Coilx_Stop(&Current_B);
	Coilx_Stop(&Current_C);
	Coilx_Stop(&Current_D);
	Current_A.Expect_Value=0U;
	Current_B.Expect_Value=0U;
	Current_C.Expect_Value=0U;
	Current_D.Expect_Value=0U;
	Current_A.PID.Sum_Err=0U;
	Current_B.PID.Sum_Err=0U;
	Current_C.PID.Sum_Err=0U;
	Current_D.PID.Sum_Err=0U;
}
/**** ***END OF FILE  ****/
