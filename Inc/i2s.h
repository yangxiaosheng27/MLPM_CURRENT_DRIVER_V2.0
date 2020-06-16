/**
  ******************************************************************************
  * File Name          : I2S.h
  * Description        : This file provides code for the configuration
  *                      of the I2S instances.
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
#ifndef __i2s_H
#define __i2s_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

	 
#define InitFilter_N	10			//初始化获取AD偏置值的滤波数据长度
#define	MAX_CURRENT		20000		//20A	
extern I2S_HandleTypeDef hi2s2;
void 	 MX_I2S2_Init(void);

/* USER CODE BEGIN Prototypes */

extern uint16_t ADS8674_RxData[8];
extern uint16_t ADS8674_TxData[8];

void	 ADS8674_Init(void);
void 	 ADS8674_GetCurrent( void );
void   Get_ADS8674_BiasValue(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ i2s_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
