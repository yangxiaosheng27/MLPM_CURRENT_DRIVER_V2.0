/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Board_DEC.h
  * @brief          : Header for Board_DEC.c file.
  *                 : �����˰�������״̬LED���ƺ����͸��ּ�Ᵽ������ 
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
	

/*���ӵ�����״̬*/	
#define 	 Standby								0x11		//����״̬
#define		 Running								0x01		//����״̬
#define		 Initial								0x02		//��ʼ��״̬
	
/*������ÿ���ؼ�������ܳ��ֵĴ���״̬*/
#define		 OverCurrent_Err				0x03		//��Ȧ����
#define		 OpenCircuit_Err				0x04		//��Ȧ��·
#define		 CanBus_Comm_Err		 		0x05		//ͨѶ����
#define		 ADC_Trans_Err_Soft			0x06		//ADCƫ��ֵ��ȡ����
#define		 ADC_Trans_Err_Hard			0x07		//ADCת��Ӳ������

#define		 Coil_Enable						0x08		//��Ȧʹ�ܣ��е������
#define		 Coil_Disable						0x00		//��Ȧʧ�ܣ��޵������
	
#define 	 X_Axis									0x00		
#define 	 Y_Axis									0x01

#define		 ADC_BIAS_VALUE					8192U		//������ADC��ƫ��ֵ
#define		 ADC_BIAS_VALUE_ERR			300U		//�����ADCƫ��ֵ��� 

#define		 OpenCircuit_MeasurementValue		10	//��·������£��ɻ����ĵ���ֵ��һ���С��10mA
#define		 TIME_200mS_5S					25			//����200ms��ʱ�м��5Sʱ���ʱ����
#define		 TIME_200mS_600mS				3				//����200ms��ʱ�м��600mSʱ���ʱ����
/**���ڼ�ذ��Ϲؼ�������������Ϣ*****/
typedef struct Info
{
	uint8_t Coil_A;
	uint8_t Coil_B;
	uint8_t Coil_C;
	uint8_t Coil_D;
	uint8_t CANBus;
	uint8_t Board_ADC;
} Board_Info;

extern uint32_t Board_State;													//���ӵ�ǰ������״̬
extern uint8_t led_BoardState;									//���ڿ��ư���״̬LED�ļ�������
extern Board_Info Board_Running_Info;									//���ڼ�ذ��Ϲؼ�������������Ϣ
extern uint32_t CANBus_DataRecieve_TimeCNT;						//����ͳ��can����û���յ����ݵ�ʱ���������
void LED_State_Init(void);

void user_Board_Initial(void);												//���ӳ�ʼ������ɰ��ӵĵ�ַ��ȡ

void Board_ADC_Bias_DEC_Soft(uint16_t Bias_A,uint16_t Bias_B,uint16_t Bias_C,uint16_t Bias_D);		//���ADCƫ��ֵ�������

void Board_ADC_Err_DEC_Hard(void);								//���ADCӲ������������

void Board_Coil_OverCurrent_DEC(void); 						// �����Ȧ�����������

void Board_Coil_OpenOpenCircuit_DEC(void); 				// �����Ȧ��·�������

void Board_CANBus_OverTime_DEC(void);							// ���CANͨѶ��ʱ�������

void Board_CanBus_DEC(int Recieve_CurrentA,int Recieve_CurrentB,int Recieve_CurrentC,int Recieve_CurrentD);	// ���CAN����ͨѶ֡����������

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
