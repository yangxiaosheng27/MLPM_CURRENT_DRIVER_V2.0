/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

uint8_t Read_AddressBit(void)
{
	uint8_t BoardAddress;
	BoardAddress = (!AddressBit0<<5)|(!AddressBit1<<4)|(!AddressBit2<<3)|(!AddressBit3<<2)|(!AddressBit4<<1)|(!AddressBit5<<0);
	return BoardAddress;
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : OUTPUT*/
  GPIO_InitStruct.Pin = BoardState_LED_Pin|AxisX_LED_Pin|AxisY_LED_Pin|CoilA_LED_Pin 
                          |CoilB_LED_Pin|CoilC_LED_Pin|CoilD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = RX485_DE_Pin|RX485_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT */
  GPIO_InitStruct.Pin = 	ADS8674_ALM_Pin|AddressBit0_Pin|AddressBit1_Pin 
													|AddressBit2_Pin|AddressBit3_Pin|AddressBit4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = 	AddressBit5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BoardState_LED_Pin|AxisX_LED_Pin|AxisY_LED_Pin|CoilA_LED_Pin \
													 |CoilB_LED_Pin|CoilC_LED_Pin|CoilD_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, RX485_DE_Pin|RX485_RE_Pin, GPIO_PIN_RESET);
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
