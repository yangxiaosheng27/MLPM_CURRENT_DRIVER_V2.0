/**
  ******************************************************************************
  * File Name          : ADS8674.h
  * Description        : This file provides the drive program  of ADS8674.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ads8674_H
#define __ads8674_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/***********Define the Registers ofADS8684*************************************/
	 
/***********Define the Registers ofADS8684*************************************/
	 
/**Command Register ***/
#define NO_OP											0X0000
#define STDBY											0X8200
#define PWR_DN  									0X8300
#define RST												0X8500
#define AUTO_RST									0XA000
	 
#define MAN_Ch_0									0XC000
#define MAN_CH_1									0XC400
#define MAN_Ch_2									0XC800
#define MAN_Ch_3									0XCC00
#define MAN_AUX										0XE000

//ADS8674 Program Registers
#define AUTO_SEQ_EN								0X01
#define Channel_Power Down 				0X02
#define Feature Select						0X03
 
#define Channel_0_Input_Range			0X05
#define Channel_1_Input_Range			0X06
#define Channel_2_Input_Range			0X07
#define Channel_3_Input_Range			0X08

//User Define


/***For SPI Bus Control ******/
#define ADS8674_START_COMMUNICATION		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_RESET)
#define ADS8674_END_COMMUNICATION			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_SET)

#define WRITE 										1
#define READ 											0

/****Stype of SPI rxData****/	
#define	WR_REG_DATA_NOT_USE				0
#define WR_CMD_DATA_NOT_USE				0
#define WR_REG_DATA								1
#define WR_CMD_DATA								2

/******ADS8674 Control******/

#define CH3_EN  									0X08
#define CH2_EN  									0X04
#define CH1_EN  									0X02
#define CH0_EN  									0X01
 
#define CH3_PD  									0X08
#define CH2_PD  									0X04
#define CH1_PD  									0X02
#define CH0_PD  									0X01
 
#define VREF_25_25								0X00
#define VREF_125_125							0X01
#define VREF_0625_0625						0X02
#define VREF_0_25									0X05
#define VREF_0_125								0X06
/***********************************************************************************************/
	 
/* Drive functions of ADS8674  ------------------------------------------------*/



void 	ADS8674_SPI_ReadWrite_xBytes(uint8_t *txData,uint8_t *rxData,uint32_t dataLength);
void 	ADS8674_SPI_Write_CommandRegister(uint32_t command);
void 	ADS8674_SPI_Write_ProgramRegister(uint32_t address, uint32_t data);
void	ADS8674_SPI_Read_ProgramRegister(uint32_t address);
void 	ADS8674_Set_Auto_RST_Mode(void);
void 	ADS8674_Set_Auto_Scan_Sequence(uint32_t sequence);
void 	ADS8674_Set_CH_Range_Select(uint32_t ch, uint32_t range);
void 	ADS8674_Get_AUTO_RST_Mode_Data(void);
void 	ADS8674_Get_Manual_Mode_Data(uint32_t ch);
void 	ADS8674_Init(void);


extern uint8_t  ADS8674_rxBuf_4BIT[4];
extern uint16_t	ADC_Value[4];

extern uint8_t  Return_Data_Format;

#ifdef __cplusplus
}
#endif

#endif /*__ads8674_H */


/**** ***END OF FILE  ****/
