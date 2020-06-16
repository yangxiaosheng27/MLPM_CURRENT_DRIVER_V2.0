/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
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
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/*显示运行状态的LED都是接的GPIOC端口*/
#define	LEDx_ON(GPIO_PinX)	HAL_GPIO_WritePin(GPIOC, GPIO_PinX, GPIO_PIN_RESET)
#define	LEDx_OFF(GPIO_PinX)	HAL_GPIO_WritePin(GPIOC, GPIO_PinX, GPIO_PIN_SET)

#define BoardState_LED_Pin GPIO_PIN_13
#define BoardState_LED_GPIO_Port GPIOC	 
#define AxisX_LED_Pin GPIO_PIN_14
#define AxisX_LED_GPIO_Port GPIOC
#define AxisY_LED_Pin GPIO_PIN_15
#define AxisY_LED_GPIO_Port GPIOC
#define CoilA_LED_Pin GPIO_PIN_0
#define CoilA_LED_GPIO_Port GPIOC
#define CoilB_LED_Pin GPIO_PIN_1
#define CoilB_LED_GPIO_Port GPIOC
#define CoilC_LED_Pin GPIO_PIN_2
#define CoilC_LED_GPIO_Port GPIOC
#define CoilD_LED_Pin GPIO_PIN_3
#define CoilD_LED_GPIO_Port GPIOC

#define TIM1_PWM1_Pin GPIO_PIN_8
#define TIM1_PWM1_GPIO_Port GPIOA
#define TIM1_PWM2_Pin GPIO_PIN_9
#define TIM1_PWM2_GPIO_Port GPIOA
#define TIM1_PWM3_Pin GPIO_PIN_10
#define TIM1_PWM3_GPIO_Port GPIOA
#define TIM1_PWM4_Pin GPIO_PIN_11
#define TIM1_PWM4_GPIO_Port GPIOA
#define TIM8_PWM1_Pin GPIO_PIN_6
#define TIM8_PWM1_GPIO_Port GPIOC
#define TIM8_PWM2_Pin GPIO_PIN_7
#define TIM8_PWM2_GPIO_Port GPIOC
#define TIM8_PWM3_Pin GPIO_PIN_8
#define TIM8_PWM3_GPIO_Port GPIOC
#define TIM8_PWM4_Pin GPIO_PIN_9
#define TIM8_PWM4_GPIO_Port GPIOC

#define TrackingDAC_OUT_A_Pin GPIO_PIN_4
#define TrackingDAC_OUT_A_GPIO_Port GPIOA
#define TrackingDAC_OUT_B_Pin GPIO_PIN_5
#define TrackingDAC_OUT_B_GPIO_Port GPIOA

#define ADS8674_ALM_Pin GPIO_PIN_11
#define ADS8674_ALM_GPIO_Port GPIOB

#define TIM3_PWM1_Pin GPIO_PIN_6
#define TIM3_PWM1_GPIO_Port GPIOC
#define TIM3_PWM2_Pin GPIO_PIN_7
#define TIM3_PWM2_GPIO_Port GPIOC
#define TIM3_PWM3_Pin GPIO_PIN_8
#define TIM3_PWM3_GPIO_Port GPIOC
#define TIM3_PWM4_Pin GPIO_PIN_9
#define TIM3_PWM4_GPIO_Port GPIOC

#define RX485_DE_Pin GPIO_PIN_0
#define RX485_DE_GPIO_Port GPIOA
#define RX485_RE_Pin GPIO_PIN_1
#define RX485_RE_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_2
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_3
#define RS485_RX_GPIO_Port GPIOA

#define AddressBit0_Pin GPIO_PIN_3
#define AddressBit0_GPIO_Port GPIOB
#define AddressBit1_Pin GPIO_PIN_4
#define AddressBit1_GPIO_Port GPIOB
#define AddressBit2_Pin GPIO_PIN_7
#define AddressBit2_GPIO_Port GPIOB
#define AddressBit3_Pin GPIO_PIN_8
#define AddressBit3_GPIO_Port GPIOB
#define AddressBit4_Pin GPIO_PIN_9
#define AddressBit4_GPIO_Port GPIOB
#define AddressBit5_Pin GPIO_PIN_2
#define AddressBit5_GPIO_Port GPIOD

#define AddressBit0 HAL_GPIO_ReadPin(AddressBit0_GPIO_Port, AddressBit0_Pin)
#define AddressBit1 HAL_GPIO_ReadPin(AddressBit1_GPIO_Port, AddressBit1_Pin)
#define AddressBit2 HAL_GPIO_ReadPin(AddressBit2_GPIO_Port, AddressBit2_Pin)
#define AddressBit3 HAL_GPIO_ReadPin(AddressBit3_GPIO_Port, AddressBit3_Pin)
#define AddressBit4 HAL_GPIO_ReadPin(AddressBit4_GPIO_Port, AddressBit4_Pin)
#define AddressBit5 HAL_GPIO_ReadPin(AddressBit5_GPIO_Port, AddressBit5_Pin)

#define ADC_AlarmPin	HAL_GPIO_ReadPin(ADS8674_ALM_GPIO_Port, ADS8674_ALM_Pin)

uint8_t Read_AddressBit(void);

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
