/**
  ******************************************************************************
  * File Name          : I2S.c
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

/* Includes ------------------------------------------------------------------*/
#include "i2s.h"

/* USER CODE BEGIN 0 */
uint16_t ADS8674_RxData[8] = {0,0,0,0,0,0,0,0};			 //接收数据，ADS8674_RxData[1,3,5,7]为ADC值返回数据（前14bit有效），ADS8674_RxData[0,2,4,6]恒为零
uint16_t ADS8674_TxData[8] = {0xA000,0,0,0,0,0,0,0}; //发送数据，ADS8674_TxData[0,2,4,6]发送给ADC的有效数据，ADS8674_TxData[1,3,5,7]可为任意数据
uint16_t ADS8674_Cmd_RST[2] = {0x8500,0}; 					 //发送给ADC的强制复位指令
/* USER CODE END 0 */


I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_i2s2_ext_rx;


void	ADS8674_Init(void)
{
	HAL_I2SEx_TransmitReceive(&hi2s2, ADS8674_Cmd_RST, ADS8674_RxData, 1, 100);
	HAL_Delay(10);
	HAL_I2SEx_TransmitReceive_DMA(&hi2s2, ADS8674_TxData, ADS8674_RxData, 4);
}

void Get_ADS8674_BiasValue(void)
{
	uint16_t j;
	uint32_t Sum_Tmp_A = 0;
	uint32_t Sum_Tmp_B = 0;
	uint32_t Sum_Tmp_C = 0;
	uint32_t Sum_Tmp_D = 0;
	HAL_Delay(0);
	for(j=0;j<InitFilter_N;j++)
	{
		HAL_Delay(1);
		Sum_Tmp_A += ADS8674_RxData[1]>>2;
		Sum_Tmp_B += ADS8674_RxData[7]>>2;
		Sum_Tmp_C += ADS8674_RxData[5]>>2;
		Sum_Tmp_D += ADS8674_RxData[3]>>2;
	}
	Current_A.Bias_Value = Sum_Tmp_A/InitFilter_N;												//在电流校正中修改电流偏移量
	Current_B.Bias_Value = Sum_Tmp_B/InitFilter_N;												//在电流校正中修改电流偏移量
	Current_C.Bias_Value = Sum_Tmp_C/InitFilter_N;												//在电流校正中修改电流偏移量
	Current_D.Bias_Value = Sum_Tmp_D/InitFilter_N;												//在电流校正中修改电流偏移量
	
	/**ADC偏置值检测**/
	Board_ADC_Bias_DEC_Soft(Current_A.Bias_Value,Current_B.Bias_Value,Current_C.Bias_Value,Current_D.Bias_Value);
}
void ADS8674_GetCurrent( void )
{
	//ADS8674是14位ADC，返回数据中前14位为有效数据，默认量程为+-10.24V，绕组电流/运放输出电压=20A/9.167V，故电流分辨率为0.002727A
	if(Board_State!=Initial)	//板子在初始化的时候不进行电流转换的计算
	{
		Current_A.Sample_Value = (int32_t)((ADS8674_RxData[1]>>2)-Current_A.Bias_Value)*2727/1000;				//电流单位：mA
		Current_B.Sample_Value = (int32_t)((ADS8674_RxData[7]>>2)-Current_B.Bias_Value)*2727/1000;
		Current_C.Sample_Value = (int32_t)((ADS8674_RxData[5]>>2)-Current_C.Bias_Value)*2727/1000;
		Current_D.Sample_Value = (int32_t)((ADS8674_RxData[3]>>2)-Current_D.Bias_Value)*2727/1000;	
		Current_A.Feedback_Value = Current_A.Sample_Value;							//应该在滤波函数中根据采样值计算反馈值
		Current_B.Feedback_Value = Current_B.Sample_Value;							//应该在滤波函数中根据采样值计算反馈值
		Current_C.Feedback_Value = Current_C.Sample_Value;							//应该在滤波函数中根据采样值计算反馈值
		Current_D.Feedback_Value = Current_D.Sample_Value;							//应该在滤波函数中根据采样值计算反馈值
	}
	else					//ADC初始化过程中反馈电流设为0
	{
		Current_A.Feedback_Value = 0;																	//应该在滤波函数中根据采样值计算反馈值
		Current_B.Feedback_Value = 0;							//应该在滤波函数中根据采样值计算反馈值
		Current_C.Feedback_Value = 0;							//应该在滤波函数中根据采样值计算反馈值
		Current_D.Feedback_Value = 0;							//应该在滤波函数中根据采样值计算反馈值
	}
}


/* I2S2 init function */
void MX_I2S2_Init(void) //启动DMA后，更新ADC的4个电流通道共需要10微秒，无需进入I2S和DMA中断，不占用CPU资源，与ADC通讯波特率为12.8MHz
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PCM_SHORT;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_32B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_192K;			//此设置位无效，在HAL_I2S_Init()中已强制赋值：i2sdiv = 3U;i2sodd = 1U;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;					//I2S_CLOCK_PLL配置成76.8MHz
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2S_MspInit(I2S_HandleTypeDef* i2sHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2sHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* I2S2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2S2 GPIO Configuration    
    PB12     ------> I2S2_WS
    PB13     ------> I2S2_CK
    PB14     ------> I2S2_ext_SD
    PB15     ------> I2S2_SD 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2S2ext;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2S2 DMA Init */
    /* SPI2_TX Init */
    hdma_spi2_tx.Instance = DMA1_Stream4;
    hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_spi2_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_spi2_tx.Init.MemBurst = DMA_MBURST_INC8;
    hdma_spi2_tx.Init.PeriphBurst = DMA_MBURST_INC8;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2sHandle,hdmatx,hdma_spi2_tx);

    /* I2S2_EXT_RX Init */
    hdma_i2s2_ext_rx.Instance = DMA1_Stream3;
    hdma_i2s2_ext_rx.Init.Channel = DMA_CHANNEL_3;
    hdma_i2s2_ext_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2s2_ext_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2s2_ext_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2s2_ext_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2_ext_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s2_ext_rx.Init.Mode = DMA_CIRCULAR;
    hdma_i2s2_ext_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2s2_ext_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_i2s2_ext_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2s2_ext_rx.Init.MemBurst = DMA_MBURST_INC8;
    hdma_i2s2_ext_rx.Init.PeriphBurst = DMA_MBURST_INC8;
    if (HAL_DMA_Init(&hdma_i2s2_ext_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2sHandle,hdmarx,hdma_i2s2_ext_rx);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_I2S_MspDeInit(I2S_HandleTypeDef* i2sHandle)
{

  if(i2sHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**I2S2 GPIO Configuration    
    PB12     ------> I2S2_WS
    PB13     ------> I2S2_CK
    PB14     ------> I2S2_ext_SD
    PB15     ------> I2S2_SD 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    /* I2S2 DMA DeInit */
    HAL_DMA_DeInit(i2sHandle->hdmatx);
    HAL_DMA_DeInit(i2sHandle->hdmarx);
  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
