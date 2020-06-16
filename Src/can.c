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

float theta1=0.0f,theta2=0.0f;					//�����������ͱ������ڴ洢CAN���߽��յ��ĵ�Ƕ��ź� ��Χ��-pi��+pi
uint16_t Iabs_1=0,Iabs_2=0;							//�������������ڴ洢CAN���߽��յ��ĵ���ģֵ�ź�		 ��Χ��0-20000mA
int CAN_Recieve_CurrentA=0;							//���ĸ��������ڴ洢���ݵ���ģֵ�͵�ǶȽ����������Ȧ����ֵ
int CAN_Recieve_CurrentB=0;
int CAN_Recieve_CurrentC=0;
int CAN_Recieve_CurrentD=0;

/******�����������******/
void Current_Demodulation_Fcn(uint16_t Iabs,float theta)
{
	CAN_Recieve_CurrentA = Iabs*sin(theta);
	CAN_Recieve_CurrentB = Iabs*sin(theta+PI/4.0f);
	CAN_Recieve_CurrentC = Iabs*sin(theta+PI/2.0f);
	CAN_Recieve_CurrentD = Iabs*sin(theta+PI*3.0f/4.0f);
	//�жϵ����Ƿ�С�ڿ�·�����ٽ���������С���ٽ�����򲻸������ָ��,���������Ʋ���Ӧ  �ٽ������10mA
	if(abs(CAN_Recieve_CurrentA)<OpenCircuit_MeasurementValue) CAN_Recieve_CurrentA=0;	
	if(abs(CAN_Recieve_CurrentB)<OpenCircuit_MeasurementValue) CAN_Recieve_CurrentB=0;
	if(abs(CAN_Recieve_CurrentC)<OpenCircuit_MeasurementValue) CAN_Recieve_CurrentC=0;
	if(abs(CAN_Recieve_CurrentD)<OpenCircuit_MeasurementValue) CAN_Recieve_CurrentD=0;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
	CANBus_DataRecieve_TimeCNT=0;		//��������ͳ��can����û���յ����ݵ�ʱ���������
	
  if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &CAN2_RxHeader, CAN2_RxData) != HAL_OK) /* Get RX message */
  {
    Error_Handler();
  }

	/********���յ�dSpace�������ֵ�����֡*******/
  if ((CAN2_RxHeader.StdId == RECIEVE_HANDSHAKE_ID) && (CAN2_RxHeader.IDE == CAN_ID_STD) && (CAN2_RxHeader.DLC == 1))
  {		
			Flag_for_HandShake=1;							//**������������ʱ��������֡����ֹ���߳�ͻ
			Flag_for_FristTime_ErrorReport=0;	//��һ�η��ʹ��󱨸��־λ��λ
			led_BoardState=0;	
		
			Board_State=Standby;
			AllCoils_Stop();
		/**����ADC�Ĵ���֮��Ĵ����ź�ȫ����λ����ΪADC�Ĵ����źſ��ܰ�����Ӳ������**/
			Board_Running_Info.Coil_A=Standby;
			Board_Running_Info.Coil_B=Standby;
			Board_Running_Info.Coil_C=Standby;
			Board_Running_Info.Coil_D=Standby;
			Board_Running_Info.CANBus=Standby;
  }
	
	/*******���յ�dSpace����ֹͣ����������֡*****/
  if ((CAN2_RxHeader.StdId == RECIEVE_STOP_ID) && (CAN2_RxHeader.IDE == CAN_ID_STD) && (CAN2_RxHeader.DLC == 1))
  {
		if(Board_State!=Standby && Board_State != Running && Board_State!=Initial)
		{	
			//*****���ӳ����ˣ������κβ���*****/
		}
		else
		{
			//*****δ��������������,�������ص�����״̬*****/
			Board_State=Standby;
			AllCoils_Stop();
		}
  }
	
	/*******���յ�dSpace���͵ĵ����ź�******/
  if ((CAN2_RxHeader.StdId == RECIEVE_CURRENT_OUTPUT_CMD_AREA1) && (CAN2_RxHeader.IDE == CAN_ID_STD) && (CAN2_RxHeader.DLC == 8))
  {
		/******��ȡCAN���յ�����******/
		Iabs_1 = (uint16_t)(CAN2_RxData[1]<<8|CAN2_RxData[0]);
		if(Iabs_1>CANBUS_RECIEVE_MAX_CURRENT) { Board_State=CanBus_Comm_Err;return; }
		theta1 = (float)((uint16_t)(CAN2_RxData[3]<<8|CAN2_RxData[2])/10000.0f)-PI;
		if(fabs(theta1)>PI+0.1f) 									{ Board_State=CanBus_Comm_Err;return; }
		Iabs_2 = (uint16_t)(CAN2_RxData[5]<<8|CAN2_RxData[4]);
		if(Iabs_2>CANBUS_RECIEVE_MAX_CURRENT)	{ Board_State=CanBus_Comm_Err;return; }
		theta2 = (float)((uint16_t)(CAN2_RxData[7]<<8|CAN2_RxData[6])/10000.0f)-PI;	
		if(fabs(theta2)>PI+0.1f) 									{ Board_State=CanBus_Comm_Err;return; }
		
		/*************������Ȧ�������ֵ***************/
		if(Board_Number>0&&Board_Number<9)
		{
				if(Standby==Board_State||Running==Board_State)	//ֻ���ڰ������л��߾���״̬�²ſ��Ըı�����ź�
				{
					Board_State=Running;
					if(Board_Number>0&&Board_Number<5)
					{
						Current_Demodulation_Fcn(Iabs_1,theta1);		//�������
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
		/******��ȡCAN���յ�����******/
		Iabs_1 = (uint16_t)(CAN2_RxData[1]<<8|CAN2_RxData[0]);
		if(Iabs_1>CANBUS_RECIEVE_MAX_CURRENT) {Board_State=CanBus_Comm_Err;return;}
		
		theta1 = (float)((uint16_t)(CAN2_RxData[3]<<8|CAN2_RxData[2])/10000.0f)-PI;
		if(fabs(theta1)>PI+0.1f) 									{Board_State=CanBus_Comm_Err;return;}
		
		Iabs_2 = (uint16_t)(CAN2_RxData[5]<<8|CAN2_RxData[4]);
		if(Iabs_2>CANBUS_RECIEVE_MAX_CURRENT)	{Board_State=CanBus_Comm_Err;return;}
		
		theta2 = (float)((uint16_t)(CAN2_RxData[7]<<8|CAN2_RxData[6])/10000.0f)-PI;	
		if(fabs(theta2)>PI+0.1f) 									{Board_State=CanBus_Comm_Err;return;}
		
		/*************������Ȧ�������ֵ***************/
		if(Board_Number>1&&Board_Number<10)
		{
				if(Standby==Board_State||Running==Board_State)	//ֻ���ڰ������л��߾���״̬�²ſ��Ըı�����ź�
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
		/******��ȡCAN���յ�����******/
		Iabs_1 = (uint16_t)(CAN2_RxData[1]<<8|CAN2_RxData[0]);
		if(Iabs_1>CANBUS_RECIEVE_MAX_CURRENT) {Board_State=CanBus_Comm_Err;return;}
		
		theta1 = (float)((uint16_t)(CAN2_RxData[3]<<8|CAN2_RxData[2])/10000.0f)-PI;		
		if(fabs(theta1)>PI+0.1f) 									{Board_State=CanBus_Comm_Err;return;}
		
		Iabs_2 = (uint16_t)(CAN2_RxData[5]<<8|CAN2_RxData[4]);
		if(Iabs_2>CANBUS_RECIEVE_MAX_CURRENT)	{Board_State=CanBus_Comm_Err;return;}
		
		theta2 = (float)((uint16_t)(CAN2_RxData[7]<<8|CAN2_RxData[6])/10000.0f)-PI;	
		if(fabs(theta2)>PI+0.1f) 									{Board_State=CanBus_Comm_Err;return;}
		
		/******End ��ȡCAN���յ�����******/
		
		/*************������Ȧ�������ֵ***************/
		if(Board_Number>2&&Board_Number<11)
		{
				if(Standby==Board_State||Running==Board_State)	//ֻ���ڰ������л��߾���״̬�²ſ��Ըı�����ź�
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
	CAN2_TxHeader.StdId = (Board_Address/100)*16*16+(Board_Address/10%10)*16+Board_Address%10;		//���÷�������֡ID ʮ����ת16���� X��101~110  Y��201~210
  CAN2_TxHeader.DLC = 1;																																				//���÷�������֡����
	HAL_Delay(Board_Number);																																			//�ӳ���Ӧ�ĵ�ʱ�䣬��֤���߲�����
	if(HAL_CAN_AddTxMessage(&hcan2, &CAN2_TxHeader, &Send_Hand_Shake_Data, &CAN2_TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler();
	}

}
/******��dSpace���������Ϣ�ĺ���*****/
void Error_Report_Tx_Fcn(void)
{
	CAN2_TxHeader.StdId = (Board_Address/100)*16*16+(Board_Address/10%10)*16+Board_Address%10;			//���÷�������֡ID ʮ����ת16���� X��101~110  Y��201~210
  CAN2_TxHeader.DLC = 1;//���÷�������֡����
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
	/*************1M������*************/
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
	
  /* ��ʼ��CAN���ߵ��˲��� */
  sFilterConfig.FilterBank = 14;											//CAN1�˲������Ϊ0,CAN2�˲������Ϊ14.
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;	//����ģʽ
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
  CAN2_TxHeader.StdId = 0x00;								//��������֡ID
  CAN2_TxHeader.ExtId = 0x01;
  CAN2_TxHeader.RTR = CAN_RTR_DATA;
  CAN2_TxHeader.IDE = CAN_ID_STD;
  CAN2_TxHeader.DLC = 1;
  CAN2_TxHeader.TransmitGlobalTime = DISABLE;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)	//CAN������ɵĻص�����
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan); 
	HAL_CAN_DeactivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY);	//�ر�CAN������ɵ��ж�
}


void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN2)
  {
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();//����ʹ��CAN1ʱ��
  
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
