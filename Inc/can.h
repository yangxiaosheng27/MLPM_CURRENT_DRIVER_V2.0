/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define 	RECIEVE_HANDSHAKE_ID		0X601		//dSpace发送的请求握手的ID
#define		RECIEVE_STOP_ID					0X001			//dSpace发送的请求驱动板停止工作的ID

#define		RECIEVE_CURRENT_OUTPUT_CMD_AREA1 0X501
#define		RECIEVE_CURRENT_OUTPUT_CMD_AREA2 0X502
#define		RECIEVE_CURRENT_OUTPUT_CMD_AREA3 0X503
	 
#define		CANBUS_RECIEVE_MAX_CURRENT	20000	//接收到的电流最大为20A
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
extern CAN_TxHeaderTypeDef CAN2_TxHeader;
extern CAN_RxHeaderTypeDef CAN2_RxHeader;
extern uint8_t  CAN2_TxData[8];
extern uint8_t  CAN2_RxData[8];
extern uint32_t CAN2_TxMailbox;
extern int CAN_Recieve_CurrentA;							//这四个变量用于存储根据电流模值和电角度解算出来的线圈电流值
extern int CAN_Recieve_CurrentB;
extern int CAN_Recieve_CurrentC;
extern int CAN_Recieve_CurrentD;
/* USER CODE END Private defines */

void MX_CAN2_Init(void);

void Error_Report_Tx_Fcn(void);

void Hand_Shake_Tx_Fcn(void);

void Current_Demodulation_Fcn(uint16_t Iabs,float theta);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
