/**
  ******************************************************************************
  * File Name          : TIM.h
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define TIM_Clock 168L	//MHz
#define PWM_Freq 100L		//kHz
#define MAX_PWM_PULSE (TIM_Clock*1000L/PWM_Freq)		//1680
#define MAX_PWM_DUTY 	(MAX_PWM_PULSE*0.95f)
#define	MAX_SUM_ERR		MAX_PWM_DUTY
	 
#define Time_200ms			10000		//200ms的定时，取决于定时器1的中断周期（20us）
	 
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern int32_t Start_Current_Recording;

/* USER CODE BEGIN Private defines */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
	 
/* USER CODE END Private defines */
void MX_TIM8_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
                        
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                        
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
