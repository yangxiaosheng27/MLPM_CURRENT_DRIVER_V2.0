/**
  ******************************************************************************
  * File Name          : ADS8674.c
  * Description        : This file provides the drive program  of ADS8674.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ads8674.h"
uint8_t  ADS8674_rxBuf_4BIT[4];
uint8_t  ADS8674_rxBuf_16BIT[16];

uint16_t	ADC_Value[4];
 
uint8_t  Return_Data_Format=0;			//for debug

void 	ADS8674_SPI_ReadWrite_xBytes(uint8_t *txData,uint8_t *rxData,uint32_t dataLength)
{
	while(hspi2.State==HAL_SPI_STATE_BUSY_TX_RX);
	ADS8674_START_COMMUNICATION;
	HAL_SPI_TransmitReceive_DMA(&hspi2,txData,rxData,dataLength);
}

void ADS8674_SPI_Write_CommandRegister(uint32_t command)
{
	uint8_t sendData_Temp[4];
	sendData_Temp[0]=(command & 0XFF00) >> 8;
	sendData_Temp[1]=command & 0X00FF;
	ADS8674_SPI_ReadWrite_xBytes(sendData_Temp,ADS8674_rxBuf_4BIT,4);
}

void ADS8674_SPI_Write_ProgramRegister(uint32_t address, uint32_t data)
{
	uint8_t sendData_Temp[4];
	sendData_Temp[0]=(address << 1) | WRITE;
	sendData_Temp[1]=data;
	ADS8674_SPI_ReadWrite_xBytes(sendData_Temp,ADS8674_rxBuf_4BIT,4);
}

void ADS8674_SPI_Read_ProgramRegister(uint32_t address)
{
	static uint8_t sendData_Temp[4];
	sendData_Temp[0]=(address) << 1 | READ;
	sendData_Temp[1]=0xFF;
	ADS8674_SPI_ReadWrite_xBytes(sendData_Temp,ADS8674_rxBuf_4BIT,4);
}

void ADS8674_Set_Auto_RST_Mode(void)
{
	Return_Data_Format=WR_CMD_DATA_NOT_USE;
	ADS8674_SPI_Write_CommandRegister(AUTO_RST);
}

void ADS8674_Set_Auto_Scan_Sequence(uint32_t sequence)
{
	Return_Data_Format=WR_REG_DATA_NOT_USE;
	ADS8674_SPI_Write_ProgramRegister(AUTO_SEQ_EN, sequence);
}

void ADS8674_Set_CH_Range_Select(uint32_t ch, uint32_t range)
{
	Return_Data_Format=WR_REG_DATA_NOT_USE;
	ADS8674_SPI_Write_ProgramRegister(ch, range);
}

void ADS8674_Get_AUTO_RST_Mode_Data(void)
{
	Return_Data_Format=WR_CMD_DATA;
	ADS8674_SPI_Write_CommandRegister(0x00);	
}

void ADS8674_Get_Manual_Mode_Data(uint32_t ch)
{
	Return_Data_Format=WR_CMD_DATA;
	ADS8674_SPI_Write_CommandRegister(ch);
}
void ADS8674_Init(void)
{
	ADS8674_Set_Auto_Scan_Sequence(0x0F);
	HAL_Delay(1);
	ADS8674_SPI_Read_ProgramRegister(AUTO_SEQ_EN);
	HAL_Delay(1);
	ADS8674_Set_Auto_RST_Mode();	
	HAL_Delay(1);

}
/**** ***END OF FILE  ****/
