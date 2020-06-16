/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Board_DEC.h
  * @brief          : Header for Board_DEC.c file.
  *                 : 包含了板子运行状态LED控制函数和各种检测保护函数 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_DEC_H
#define __BOARD_DEC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
	

/*板子的运行状态*/	
#define 	 Standby								0x11		//就绪状态
#define		 Running								0x01		//驱动状态
#define		 Initial								0x02		//初始化状态
	
/*板子上每个关键任务可能出现的错误状态*/
#define		 OverCurrent_Err				0x03		//线圈过流
#define		 OpenCircuit_Err				0x04		//线圈开路
#define		 CanBus_Comm_Err		 		0x05		//通讯错误
#define		 ADC_Trans_Err_Soft			0x06		//ADC偏置值获取错误
#define		 ADC_Trans_Err_Hard			0x07		//ADC转换硬件错误

#define		 Coil_Enable						0x08		//线圈使能：有电流输出
#define		 Coil_Disable						0x00		//线圈失能：无电流输出
	
#define 	 X_Axis									0x00		
#define 	 Y_Axis									0x01

#define		 ADC_BIAS_VALUE					8192U		//理论上ADC的偏置值
#define		 ADC_BIAS_VALUE_ERR			300U		//允许的ADC偏置值误差 

#define		 OpenCircuit_MeasurementValue		10	//开路的情况下，采回来的电流值会一般会小于10mA
#define		 TIME_200mS_5S					25			//用在200ms定时中间的5S时间计时变量
#define		 TIME_200mS_600mS				3				//用在200ms定时中间的600mS时间计时变量
/**用于监控板上关键器件的运行信息*****/
typedef struct Info
{
	uint8_t Coil_A;
	uint8_t Coil_B;
	uint8_t Coil_C;
	uint8_t Coil_D;
	uint8_t CANBus;
	uint8_t Board_ADC;
} Board_Info;

extern uint32_t Board_State;													//板子当前的运行状态
extern uint8_t led_BoardState;									//用于控制板子状态LED的计数变量
extern Board_Info Board_Running_Info;									//用于监控板上关键器件的运行信息
extern uint32_t CANBus_DataRecieve_TimeCNT;						//用于统计can总线没有收到数据的时间计数变量
void LED_State_Init(void);

void user_Board_Initial(void);												//板子初始化：完成板子的地址获取

void Board_ADC_Bias_DEC_Soft(uint16_t Bias_A,uint16_t Bias_B,uint16_t Bias_C,uint16_t Bias_D);		//完成ADC偏置值检测任务

void Board_ADC_Err_DEC_Hard(void);								//完成ADC硬件错误检测任务

void Board_Coil_OverCurrent_DEC(void); 						// 完成线圈过流检测任务

void Board_Coil_OpenOpenCircuit_DEC(void); 				// 完成线圈开路检测任务

void Board_CANBus_OverTime_DEC(void);							// 完成CAN通讯超时检测任务

void Board_CanBus_DEC(int Recieve_CurrentA,int Recieve_CurrentB,int Recieve_CurrentC,int Recieve_CurrentD);	// 完成CAN总线通讯帧错误检测任务

void LEDx_Flashing_Operation(uint16_t LEDx_Pin, uint8_t *sequence, uint8_t index);

void Running_CoilxLED_State_Update(Board_Info	Board_Running_Info);

void OverCurrent_CoilxLED_State_Update(Board_Info Board_Running_Info);

void OpenCircuit_CoilxLED_State_Update(Board_Info Board_Running_Info);

void BoardLED_State_Update(uint32_t Board_State,Board_Info Board_Running_Info);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_DEC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
