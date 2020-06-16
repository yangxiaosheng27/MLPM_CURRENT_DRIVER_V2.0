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

#define DOUBLE_POLE						1		//˫����PWM
#define SINGLE_POLE						2		//������PWM
	 
#define PWM_POLE  SINGLE_POLE			//ʹ�õ�����PWM!!!

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
//PID���Ʋ����ṹ��
typedef struct
{
	f32 Kp;									//P����
	f32 Ki;									//I����
	f32 Kd;									//D����
	int32_t Err_Now;				//��ǰʱ�����
	int32_t Err_Last;				//��һʱ�����
	int32_t dErr;						//����ʱ�����Ĳ��ֵ
	int32_t Sum_Err;				//�ۻ����
	int32_t Max_SumErr;			//�ۻ�����޷�
	int32_t Err_Threshold;	//�ж�ʹ�ܻ��ֿ��Ƶ�����ٽ�ֵ
	int32_t Output;					//���������
	int32_t Max_Output;			//����������޷�
} PID_Struct;
	 
// �������Ʋ����ṹ��
typedef struct
{
	int32_t Expect_Value;									//��������ֵ(mA)
	int32_t Feedback_Value;								//��������ֵ(mA)
	int32_t Sample_Value;									//ADCֵ�������õ���ԭʼ����ֵ(mA)��δ�˲�����
	uint16_t Bias_Value;									//����ƫ����:����Ϊ��ʱ��ADCֵ
	int32_t PWM_Duty;											//PWMռ�ձȣ������ɸ�
	int32_t Max_PWM_Duty;									//PWMռ�ձ�����
	__IO uint32_t *PWM_Register_Left;			//������������ű�PWMռ�ձȵļĴ���
	__IO uint32_t *PWM_Register_Right;		//�����Ұ������ű�PWMռ�ձȵļĴ���
	PID_Struct PID;												//PID���Ʋ���

} Current_Struct;

extern PID_Struct PID_A;	 
extern PID_Struct PID_B;	 
extern PID_Struct PID_C;
extern PID_Struct PID_D;	 

extern Current_Struct Current_A;
extern Current_Struct Current_B;
extern Current_Struct Current_C;
extern Current_Struct Current_D;

void Current_Controller_Init(void);						  //���Ʋ�����ʼ��

void Current_Loop_Control(uint32_t Board_State);//����������

void PWM_Duty_Updata(Current_Struct *Current);//����PWMռ�ձ���ؼĴ���

void Controller_Parameter_Init(Current_Struct *Current, f32 Kp, f32 Ki, f32 Kd, 					\
																int32_t Max_PID_SumErr, int32_t Max_PID_Output, 	\
																int32_t Err_Threshold, int32_t Max_PWM_Duty,		\
																__IO uint32_t *PWM_Register_Left,					\
																__IO uint32_t *PWM_Register_Right);

int32_t PID_Calculate(PID_Struct *PID, int32_t Expect_Value, int32_t Feedback_Value);//����PID������

void Coilx_PWM_Duty_Update(Current_Struct *Current);
void Coilx_Stop(Current_Struct *Current);
void AllCoils_Stop(void);
static int32_t PID_Calculate(PID_Struct *PID, int32_t Expect_Value, int32_t Feedback_Value);

#ifdef __cplusplus
}
#endif

#endif /*__control_H */


/**** ***END OF FILE  ****/
