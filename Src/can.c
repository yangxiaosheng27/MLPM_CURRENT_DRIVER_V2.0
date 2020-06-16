/**
  ******************************************************************************
  * File Name          : CAN.c
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef CAN2_TxHeader;
CAN_RxHeaderTypeDef CAN2_RxHeader;
CAN_FilterTypeDef  sFilterConfig;

uint8_t  Send_Hand_Shake_Data=0x02;
uint8_t  Send_Error_Report_Data=0x03;

uint8_t  CAN2_RxData[8];

uint32_t CAN2_TxMailbox;

float theta1=0.0f,theta2=0.0f;					//这两个浮点型变量用于存储CAN总线接收到的电角度信号 范围在-pi到+pi
uint16_t Iabs_1=0,Iabs_2=0;							//这两个变量用于存储CAN总线接收到的电流模值信号		 范围在0-20000mA
int CAN_Recieve_CurrentA=0;							//这四个变量用于存储根据电流模值和电角度解算出来的线圈电流值
int CAN_Recieve_CurrentB=0;
int CAN_Recieve_CurrentC=0;
int CAN_Recieve_CurrentD=0;

/******电流解调函数******/
void Current_Demodulation_Fcn(uint16_t Iabs,float theta)
{
	CAN_Recieve_CurrentA = Iabs*sin(theta);
	CAN_Recieve_CurrentB = Iabs*sin(theta+PI/4.0f);
	CAN_Recieve_CurrentC = Iabs*sin(theta+PI/2.0f);
	CAN_Recieve_CurrentD = Iabs*sin(theta+PI*3.0f/4.0f);
	//判断电流是否小于开路检测的临界电流，如果小于临界电流则不给予控制指令,电流环控制不响应  临界电流：10mA
	if(abs(CAN_Recieve_CurrentA)<OpenCircuit_MeasurementValue) CAN_Recieve_CurrentA=0;	
	if(abs(CAN_Recieve_CurrentB)<OpenCircuit_MeasurementValue) CAN_Recieve_CurrentB=0;
	if(abs(CAN_Recieve_CurrentC)<OpenCircuit_MeasurementValue) CAN_Recieve_CurrentC=0;
	if(abs(CAN_Recieve_CurrentD)<OpenCircuit_MeasurementValue) CAN_Recieve_CurrentD=0;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
	CANBus_DataRecieve_TimeCNT=0;		//清零用于统计can总线没有收到数据的时间计数变量
	
  if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &CAN2_RxHeader, CAN2_RxData) != HAL_OK) /* Get RX message */
  {
    Error_Handler();
  }

	/********接收到dSpace请求握手的数据帧*******/
  if ((CAN2_RxHeader.StdId == RECIEVE_HANDSHAKE_ID) && (CAN2_RxHeader.IDE == CAN_ID_STD) && (CAN2_RxHeader.DLC == 1))
  {		
			Flag_for_HandShake=1;							//**在主函数中延时发送数据帧，防止总线冲突
			Flag_for_FristTime_ErrorReport=0;	//第一次发送错误报告标志位置位
			led_BoardState=0;	
		
			Board_State=Standby;
			AllCoils_Stop();
		/**除了ADC的错误之外的错误信号全部复位，因为ADC的错误信号可能包含了硬件错误**/
			Board_Running_Info.Coil_A=Standby;
			Board_Running_Info.Coil_B=Standby;
			Board_Running_Info.Coil_C=Standby;
			Board_Running_Info.Coil_D=Standby;
			Board_Running_Info.CANBus=Standby;
  }
	
	/*******接收到dSpace请求停止工作的数据帧*****/
  if ((CAN2_RxHeader.StdId == RECIEVE_STOP_ID) && (CAN2_RxHeader.IDE == CAN_ID_STD) && (CAN2_RxHeader.DLC == 1))
  {
		if(Board_State!=Standby && Board_State != Running && Board_State!=Initial)
		{	
			//*****板子出错了，不做任何操作*****/
		}
		else
		{
			//*****未出错的驱动板掉电,驱动器回到就绪状态*****/
			Board_State=Standby;
			AllCoils_Stop();
		}
  }
	
	/*******接收到dSpace发送的电流信号******/
  if ((CAN2_RxHeader.StdId == RECIEVE_CURRENT_OUTPUT_CMD_AREA1) && (CAN2_RxHeader.IDE == CAN_ID_STD) && (CAN2_RxHeader.DLC == 8))
  {
		/******获取CAN接收的数据******/
		Iabs_1 = (uint16_t)(CAN2_RxData[1]<<8|CAN2_RxData[0]);
		if(Iabs_1>CANBUS_RECIEVE_MAX_CURRENT) { Board_State=CanBus_Comm_Err;return; }
		theta1 = (float)((uint16_t)(CAN2_RxData[3]<<8|CAN2_RxData[2])/10000.0f)-PI;
		if(fabs(theta1)>PI+0.1f) 									{ Board_State=CanBus_Comm_Err;return; }
		Iabs_2 = (uint16_t)(CAN2_RxData[5]<<8|CAN2_RxData[4]);
		if(Iabs_2>CANBUS_RECIEVE_MAX_CURRENT)	{ Board_State=CanBus_Comm_Err;return; }
		theta2 = (float)((uint16_t)(CAN2_RxData[7]<<8|CAN2_RxData[6])/10000.0f)-PI;	
		if(fabs(theta2)>PI+0.1f) 									{ Board_State=CanBus_Comm_Err;return; }
		
		/*************更改线圈输出电流值***************/
		if(Board_Number>0&&Board_Number<9)
		{
				if(Standby==Board_State||Running==Board_State)	//只有在板子运行或者就绪状态下才可以改变电流信号
				{
					Board_State=Running;
					if(Board_Number>0&&Board_Number<5)
					{
						Current_Demodulation_Fcn(Iabs_1,theta1);		//电流解调
					}
					else
					{
						Current_Demodulation_Fcn(Iabs_2,theta2);
					}
					Board_CanBus_DEC(CAN_Recieve_CurrentA,CAN_Recieve_CurrentB,CAN_Recieve_CurrentC,CAN_Recieve_CurrentD);
				}
		}
		else
		{
					Board_State=Standby;
					AllCoils_Stop();
		}
  }
	 
	if ((CAN2_RxHeader.StdId == RECIEVE_CURRENT_OUTPUT_CMD_AREA2) && (CAN2_RxHeader.IDE == CAN_ID_STD) && (CAN2_RxHeader.DLC == 8))
  {
		/******获取CAN接收的数据******/
		Iabs_1 = (uint16_t)(CAN2_RxData[1]<<8|CAN2_RxData[0]);
		if(Iabs_1>CANBUS_RECIEVE_MAX_CURRENT) {Board_State=CanBus_Comm_Err;return;}
		
		theta1 = (float)((uint16_t)(CAN2_RxData[3]<<8|CAN2_RxData[2])/10000.0f)-PI;
		if(fabs(theta1)>PI+0.1f) 									{Board_State=CanBus_Comm_Err;return;}
		
		Iabs_2 = (uint16_t)(CAN2_RxData[5]<<8|CAN2_RxData[4]);
		if(Iabs_2>CANBUS_RECIEVE_MAX_CURRENT)	{Board_State=CanBus_Comm_Err;return;}
		
		theta2 = (float)((uint16_t)(CAN2_RxData[7]<<8|CAN2_RxData[6])/10000.0f)-PI;	
		if(fabs(theta2)>PI+0.1f) 									{Board_State=CanBus_Comm_Err;return;}
		
		/*************更改线圈输出电流值***************/
		if(Board_Number>1&&Board_Number<10)
		{
				if(Standby==Board_State||Running==Board_State)	//只有在板子运行或者就绪状态下才可以改变电流信号
				{
					Board_State=Running;
					if(Board_Number>1&&Board_Number<6)
					{
						Current_Demodulation_Fcn(Iabs_1,theta1);
					}
					else
					{
						Current_Demodulation_Fcn(Iabs_2,theta2);
					}
					Board_CanBus_DEC(CAN_Recieve_CurrentA,CAN_Recieve_CurrentB,CAN_Recieve_CurrentC,CAN_Recieve_CurrentD);
				}
		}
		else
		{
					Board_State=Standby;
					AllCoils_Stop();
		}
  } 
	
	if ((CAN2_RxHeader.StdId == RECIEVE_CURRENT_OUTPUT_CMD_AREA3) && (CAN2_RxHeader.IDE == CAN_ID_STD) && (CAN2_RxHeader.DLC == 8))
  {
		/******获取CAN接收的数据******/
		Iabs_1 = (uint16_t)(CAN2_RxData[1]<<8|CAN2_RxData[0]);
		if(Iabs_1>CANBUS_RECIEVE_MAX_CURRENT) {Board_State=CanBus_Comm_Err;return;}
		
		theta1 = (float)((uint16_t)(CAN2_RxData[3]<<8|CAN2_RxData[2])/10000.0f)-PI;		
		if(fabs(theta1)>PI+0.1f) 									{Board_State=CanBus_Comm_Err;return;}
		
		Iabs_2 = (uint16_t)(CAN2_RxData[5]<<8|CAN2_RxData[4]);
		if(Iabs_2>CANBUS_RECIEVE_MAX_CURRENT)	{Board_State=CanBus_Comm_Err;return;}
		
		theta2 = (float)((uint16_t)(CAN2_RxData[7]<<8|CAN2_RxData[6])/10000.0f)-PI;	
		if(fabs(theta2)>PI+0.1f) 									{Board_State=CanBus_Comm_Err;return;}
		
		/******End 获取CAN接收的数据******/
		
		/*************更改线圈输出电流值***************/
		if(Board_Number>2&&Board_Number<11)
		{
				if(Standby==Board_State||Running==Board_State)	//只有在板子运行或者就绪状态下才可以改变电流信号
				{
					Board_State=Running;
					if(Board_Number>2&&Board_Number<7)
					{
						Current_Demodulation_Fcn(Iabs_1,theta1);
					}
					else
					{
						Current_Demodulation_Fcn(Iabs_2,theta2);
					}
					Board_CanBus_DEC(CAN_Recieve_CurrentA,CAN_Recieve_CurrentB,CAN_Recieve_CurrentC,CAN_Recieve_CurrentD);
				}
		}
		else
		{
					Board_State=Standby;
					AllCoils_Stop();
		}
  }
}
void Hand_Shake_Tx_Fcn(void)
{
	CAN2_TxHeader.StdId = (Board_Address/100)*16*16+(Board_Address/10%10)*16+Board_Address%10;		//设置发送数据帧ID 十进制转16进制 X轴101~110  Y轴201~210
  CAN2_TxHeader.DLC = 1;																																				//设置发送数据帧长度
	HAL_Delay(Board_Number);																																			//延迟相应的的时间，保证总线不阻塞
	if(HAL_CAN_AddTxMessage(&hcan2, &CAN2_TxHeader, &Send_Hand_Shake_Data, &CAN2_TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler();
	}

}
/******向dSpace报告错误信息的函数*****/
void Error_Report_Tx_Fcn(void)
{
	CAN2_TxHeader.StdId = (Board_Address/100)*16*16+(Board_Address/10%10)*16+Board_Address%10;			//设置发送数据帧ID 十进制转16进制 X轴101~110  Y轴201~210
  CAN2_TxHeader.DLC = 1;//设置发送数据帧长度
	HAL_Delay(Board_Number);
	if (HAL_CAN_AddTxMessage(&hcan2, &CAN2_TxHeader, &Send_Error_Report_Data, &CAN2_TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}
}
/* CAN2 init function */
void MX_CAN2_Init(void)
{	
  hcan2.Instance = CAN2;
	/*************1M波特率*************/
  hcan2.Init.Prescaler = 2;								
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_6TQ;
	/*********************************/
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
	
  hcan2.Init.ReceiveFifoLocked = ENABLE;
  hcan2.Init.TransmitFifoPriority = ENABLE;
	
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
	
  /* 初始化CAN总线的滤波器 */
  sFilterConfig.FilterBank = 14;											//CAN1滤波器组号为0,CAN2滤波器组号为14.
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;	//掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
 
  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
  /* Activate CAN RX notification */
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
	
  /* Activate CAN TX notification */
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
	   /* Start the CAN peripheral */
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  } 
  /* Configure Transmission process */
  CAN2_TxHeader.StdId = 0x00;								//设置数据帧ID
  CAN2_TxHeader.ExtId = 0x01;
  CAN2_TxHeader.RTR = CAN_RTR_DATA;
  CAN2_TxHeader.IDE = CAN_ID_STD;
  CAN2_TxHeader.DLC = 1;
  CAN2_TxHeader.TransmitGlobalTime = DISABLE;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)	//CAN发送完成的回调函数
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan); 
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY);	//关闭CAN发送完成的中断
}


void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN2)
  {
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();//必须使能CAN1时钟
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration    
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);		
		HAL_NVIC_SetPriority(CAN2_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */
  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{
  if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
  
    /**CAN2 GPIO Configuration    
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }	
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
