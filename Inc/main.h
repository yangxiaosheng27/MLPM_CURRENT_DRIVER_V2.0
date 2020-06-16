/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dac.h"
#include "dma.h"
#include "i2s.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "clock.h"
	
/*user Includes	--------------------------------------------------------------*/
#include "control.h"
#include "Board_DEC.h"	
#include "pvd.h"
#include "stdlib.h"
#include "math.h"
/*user typedefine	------------------------------------------------------------*/	 

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

extern uint8_t Board_Address;
extern uint8_t Board_Axis;
extern uint8_t Board_Number;
extern uint8_t Flag_for_FristTime_ErrorReport;
extern uint8_t Flag_for_HandShake;
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
