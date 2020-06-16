/**
  ******************************************************************************
  * File Name          : Board_DEC.c
  * Description        : This file provides code for the Board Detection
  *                     
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Board_DEC.h"
/* USER CODE BEGIN 0 */

uint8_t Board_Address;		//���ӵĵ�ַ��Ϣ
uint8_t Board_Axis;				//���������ĸ���	
uint8_t Board_Number;			//���ӵ����

uint8_t led_BoardState;									//���ڿ��ư���״̬LED�ļ�������
uint32_t Board_State = Initial;					//���ӵ�ǰ������״̬
Board_Info Board_Running_Info;					//���ڼ�ذ��Ϲؼ�������������Ϣ
uint32_t CANBus_DataRecieve_TimeCNT;		//����ͳ��can����û���յ����ݵ�ʱ���������
/* USER CODE END 0 */
static uint8_t Sequence_FlashingQuick_Twice[7]={1,0,1,0,0,0,0};						//LED������˸���εĸ�ֵ���� didi~didi~didi~
static uint8_t Sequence_FlashingQuick_Thrice[9]={1,0,1,0,1,0,0,0,0};			//LED������˸���εĸ�ֵ����	dididi~dididi~dididi~
static uint8_t Sequence_FlashingSlow_Repeated[6]={1,1,1,0,0,0};						//LED������˸�ĸ�ֵ����	di~di~di~	
static uint8_t Sequence_FlashingQuick_Repeated[4]={1,0,1,0};							//LED������˸�ĸ�ֵ����	di-di-di-	
/* USER CODE BEGIN 1 */

/*****��ʼ��--�ر�ȫ��LED*****/
void LED_State_Init(void)
{
	LEDx_OFF(BoardState_LED_Pin);
	LEDx_OFF(AxisX_LED_Pin);
	LEDx_OFF(AxisY_LED_Pin);
	LEDx_OFF(CoilA_LED_Pin);
	LEDx_OFF(CoilB_LED_Pin);
	LEDx_OFF(CoilC_LED_Pin);
	LEDx_OFF(CoilD_LED_Pin);	
}
/****���ӳ�ʼ��������ɰ��ӵĵ�ַ��ȡ�����ADCƫ�õ�����ȡ**********
*****Note:�ó������ʹ����ʱ������ʱ�ڼ����ж����ADCƫ�õ����Ľ����ͼ��
******/
void user_Board_Initial(void)
{
	uint8_t Address=0;
	LED_State_Init();													//�ر�����LED
	Board_State	=	Initial;									//����״̬Ϊ��ʼ��״̬
	Address	=	Read_AddressBit();						//ON����1��OFF����0
	Board_Axis		=	Address&0x01;						//00 0001	���λ�ǰ��ӵ����ţ�0--X��		1--Y��
	Board_Number	=	Address>>1&0x1F;				//11 1110	����λ�ǰ��ӵ���ţ�0~9
	
	if(X_Axis==Board_Axis)	
	{
	 Board_Address = 100+Board_Number;
	 LEDx_ON(AxisX_LED_Pin);	
	}
	if(Y_Axis==Board_Axis)
	{
	Board_Address = 200+Board_Number;
	 LEDx_ON(AxisY_LED_Pin);
	}
	
	HAL_Delay(100);														//��ʱ100ms�����㹻��֤�����ϵĵ�Դȫ���ȶ�����
	
	Get_ADS8674_BiasValue();									//ADCƫ��ֵ��ȡ-���ӵ�״̬��ADC״̬ȷ��
		
	Board_Running_Info.CANBus 	 = 	Standby;
	Board_Running_Info.Coil_A 	 = 	Standby;
	Board_Running_Info.Coil_B 	 = 	Standby;
	Board_Running_Info.Coil_C 	 = 	Standby;
	Board_Running_Info.Coil_D 	 = 	Standby;
}
/**********
//ADC�����⺯�������ADCƫ��ֵУ��ͼ������
******/
void Board_ADC_Bias_DEC_Soft(uint16_t Bias_A,uint16_t Bias_B,uint16_t Bias_C,uint16_t Bias_D)		
{
	if(abs(Bias_A-ADC_BIAS_VALUE)>ADC_BIAS_VALUE_ERR || \
		 abs(Bias_B-ADC_BIAS_VALUE)>ADC_BIAS_VALUE_ERR || \
		 abs(Bias_C-ADC_BIAS_VALUE)>ADC_BIAS_VALUE_ERR || \
		 abs(Bias_D-ADC_BIAS_VALUE)>ADC_BIAS_VALUE_ERR )
	{ 
			Board_State=ADC_Trans_Err_Soft;
			Board_Running_Info.Board_ADC=ADC_Trans_Err_Soft;
	}
	else
	{
			Board_State=Standby;
			Board_Running_Info.Board_ADC=Standby;
	}
}
/**********
//ADCӲ�������⺯�������ADCӲ������������
******/
void	Board_ADC_Err_DEC_Hard(void)
{
	if(!ADC_AlarmPin)
	{
			Board_State=ADC_Trans_Err_Hard;
			Board_Running_Info.Board_ADC=ADC_Trans_Err_Hard;
	}
}
/**********
//�����Ȧ�����������
******/
void Board_Coil_OverCurrent_DEC() 		
{
	/***********�������**********/
	if(Current_A.Sample_Value>MAX_CURRENT||Current_A.Sample_Value<-MAX_CURRENT)
	{
			Board_State=OverCurrent_Err;
			Board_Running_Info.Coil_A=OverCurrent_Err;		
	}
	if(Current_B.Sample_Value>MAX_CURRENT||Current_B.Sample_Value<-MAX_CURRENT)
	{
			Board_State=OverCurrent_Err;
			Board_Running_Info.Coil_B=OverCurrent_Err;		
	}
	if(Current_C.Sample_Value>MAX_CURRENT||Current_C.Sample_Value<-MAX_CURRENT)
	{
			Board_State=OverCurrent_Err;
			Board_Running_Info.Coil_C=OverCurrent_Err;		
	}
	if(Current_D.Sample_Value>MAX_CURRENT||Current_D.Sample_Value<-MAX_CURRENT)
	{
			Board_State=OverCurrent_Err;
			Board_Running_Info.Coil_D=OverCurrent_Err;		
	}
	/*******END �������**********/
}
/**********
//��Ȧ��·��⺯��������Ƶ�� 5Hz
******/
void Board_Coil_OpenOpenCircuit_DEC(void)		
{
	/*��Ȧ��·���ԭ�����ӽ��յ���Ȧ�����źţ���running״̬�£���ʱ�����Ȧ�����Ƿ�Ϊ0���߽ӽ�0��
										 ��ʱ���⵽��Ȧ���Ϊ0����֤��������������Ч����Ȧ��·*/
	static uint8_t OP_TimeCnt_A,OP_TimeCnt_B,OP_TimeCnt_C,OP_TimeCnt_D;							//���ʱ�����
	if(Board_State==Running)
	{
/*******************************************************************************************/	
		if(Coil_Enable==Board_Running_Info.Coil_A) //���յ���A��Ȧ�ĵ����źţ�A��Ȧ�������Ѿ�ʹ��
		{
			if(abs(Current_A.Feedback_Value)<OpenCircuit_MeasurementValue)		//
			{
				OP_TimeCnt_A++;
			}
			else
			{
				OP_TimeCnt_A=0;
			}
		}
		else OP_TimeCnt_A=0;
/*******************************************************************************************/		
		if(Coil_Enable==Board_Running_Info.Coil_B) //���յ���B��Ȧ�ĵ����źţ�B��Ȧ�������Ѿ�ʹ��
		{
			if(abs(Current_B.Feedback_Value)<OpenCircuit_MeasurementValue)		//����������ʵ�ʵ������500mA:Ҫע�⣬������ܻ����bug�����������ڸ�Ƶ�����ʱ����ܻ�����
			{
				OP_TimeCnt_B++;
			}
			else
			{
				OP_TimeCnt_B=0;
			}
		}
		else OP_TimeCnt_B=0;
/*******************************************************************************************/		
		if(Coil_Enable==Board_Running_Info.Coil_C) //���յ���C��Ȧ�ĵ����źţ�C��Ȧ�������Ѿ�ʹ��
		{
			if(abs(Current_C.Feedback_Value)<OpenCircuit_MeasurementValue)		//����������ʵ�ʵ������500mA:Ҫע�⣬������ܻ����bug�����������ڸ�Ƶ�����ʱ����ܻ�����
			{
				OP_TimeCnt_C++;
			}
			else
			{
				OP_TimeCnt_C=0;
			}
		}
		else OP_TimeCnt_C=0;
/*******************************************************************************************/		
		if(Coil_Enable==Board_Running_Info.Coil_D) //���յ���D��Ȧ�ĵ����źţ�D��Ȧ�������Ѿ�ʹ��
		{
			if(abs(Current_D.Feedback_Value)<OpenCircuit_MeasurementValue)		//����������ʵ�ʵ������500mA:Ҫע�⣬������ܻ����bug�����������ڸ�Ƶ�����ʱ����ܻ�����
			{
				OP_TimeCnt_D++;
			}
			else
			{
				OP_TimeCnt_D=0;
			}
		}
		else OP_TimeCnt_D=0;
	}
	else
	{
		OP_TimeCnt_A=0;OP_TimeCnt_B=0;OP_TimeCnt_C=0;OP_TimeCnt_D=0;
	}
	
	if(OP_TimeCnt_A>TIME_200mS_5S)
	{
		Board_Running_Info.Coil_A=OpenCircuit_Err;
		if(Board_State==Standby||Running)
		{
			Board_State=OpenCircuit_Err;
		}

	}
	if(OP_TimeCnt_B>TIME_200mS_5S)
	{
			Board_Running_Info.Coil_B=OpenCircuit_Err;
		if(Board_State==Standby||Running)
		{
			Board_State=OpenCircuit_Err;
		};
	}
	if(OP_TimeCnt_C>TIME_200mS_5S)
	{
			Board_Running_Info.Coil_C=OpenCircuit_Err;
		if(Board_State==Standby||Running)
		{
			Board_State=OpenCircuit_Err;
		}
	}
	if(OP_TimeCnt_D>TIME_200mS_5S)
	{
			Board_Running_Info.Coil_D=OpenCircuit_Err;
		if(Board_State==Standby||Running)
		{
			Board_State=OpenCircuit_Err;
		}
	}
}
/**********
//CAN�����źŲ������������������ݳ����涨�������������ӱ������û��͸�������ָ�������ź�,���Ҹı���������Ʋ���
******/
void Board_CanBus_DEC(int Recieve_CurrentA,int Recieve_CurrentB,int Recieve_CurrentC,int Recieve_CurrentD)
{
	/**********CoilA************/
	if(0!=Recieve_CurrentA)	//���յ��ĵ���ָ�Ϊ0
	{
		Board_Running_Info.Coil_A=Coil_Enable;
	}
	if(abs(Recieve_CurrentA)>CANBUS_RECIEVE_MAX_CURRENT)//���յ��ĵ���ָ�����ָ�������ֵ
	{
		Board_State=CanBus_Comm_Err;
		Board_Running_Info.Coil_A=Coil_Disable;
	}
	else 
	{
		Current_A.Expect_Value = Recieve_CurrentA;
		Current_A.PID.Ki = Initial_Ki+fabs(Recieve_CurrentA*0.000006f);//���ֲ�����0-15A��������
	}
	/**********CoilB************/
	if(0!=Recieve_CurrentB)	//���յ��ĵ���ָ�Ϊ0
	{
		Board_Running_Info.Coil_B=Coil_Enable;
	}
	if(abs(Recieve_CurrentB)>CANBUS_RECIEVE_MAX_CURRENT)//���յ��ĵ���ָ�����ָ�������ֵ
	{
		Board_State=CanBus_Comm_Err;
		Board_Running_Info.Coil_B=Coil_Disable;
	}
	else 
	{
		Current_B.Expect_Value=Recieve_CurrentB;
		Current_B.PID.Ki = Initial_Ki+fabs(Recieve_CurrentB*0.000006f);//���ֲ�����0-15A��������
	}
	/**********CoilC************/	
		if(0!=Recieve_CurrentC)	//���յ��ĵ���ָ�Ϊ0
	{
		Board_Running_Info.Coil_C=Coil_Enable;
	}
	if(abs(Recieve_CurrentC)>CANBUS_RECIEVE_MAX_CURRENT)//���յ��ĵ���ָ�����ָ�������ֵ
	{
		Board_State=CanBus_Comm_Err;
		Board_Running_Info.Coil_C=Coil_Disable;
	}
	else 
	{
		Current_C.Expect_Value=Recieve_CurrentC;
		Current_C.PID.Ki = Initial_Ki+fabs(Recieve_CurrentC*0.000006f);//���ֲ�����0-15A��������
	}
	
	/**********CoilD************/	
		if(0!=Recieve_CurrentD)	//���յ��ĵ���ָ�Ϊ0
	{
		Board_Running_Info.Coil_D=Coil_Enable;
	}
	if(abs(Recieve_CurrentD)>CANBUS_RECIEVE_MAX_CURRENT)//���յ��ĵ���ָ�����ָ�������ֵ
	{
		Board_State=CanBus_Comm_Err;
		Board_Running_Info.Coil_D=Coil_Disable;
	}
	else 
	{
		Current_D.Expect_Value=Recieve_CurrentD;
		Current_D.PID.Ki = Initial_Ki+fabs(Recieve_CurrentD*0.000006f);//���ֲ�����0-15A��������
	}
	
}
/**********
//CAN�����źż�⺯��������һ��ʱ��û�н��յ�CAN���ߵ�����֮�󣬰��ӻָ���Standby״̬
******/
void Board_CANBus_OverTime_DEC(void)		
{
	if(Board_State==Running)
	{
		if(++CANBus_DataRecieve_TimeCNT>TIME_200mS_600mS)
		{
			Board_State=Standby;
			AllCoils_Stop();
		}
	}
}
/*****LED��˸���ƺ���*******/
void LEDx_Flashing_Operation(uint16_t LEDx_Pin, uint8_t *sequence, uint8_t index)
{	
	 if(sequence[index]) 
	 {
		 LEDx_ON(LEDx_Pin);
	 }
	 else 
	 {
		 LEDx_OFF(LEDx_Pin);
	 }
}
/********��������ʱ��Ȧ��ָʾ�Ƹ��º���*******/
void Running_CoilxLED_State_Update(Board_Info	Board_Running_Info)
{
		if(Coil_Enable==Board_Running_Info.Coil_A) LEDx_ON(CoilA_LED_Pin);												//���յ���ȦA�ĵ�������
		else LEDx_OFF(CoilA_LED_Pin);
		if(Coil_Enable==Board_Running_Info.Coil_B) LEDx_ON(CoilB_LED_Pin);												//���յ���ȦB�ĵ�������
		else LEDx_OFF(CoilB_LED_Pin);
		if(Coil_Enable==Board_Running_Info.Coil_C) LEDx_ON(CoilC_LED_Pin);												//���յ���ȦC�ĵ�������
		else LEDx_OFF(CoilC_LED_Pin);
		if(Coil_Enable==Board_Running_Info.Coil_D) LEDx_ON(CoilD_LED_Pin);												//���յ���ȦD�ĵ�������
		else LEDx_OFF(CoilD_LED_Pin);
}
/********����ʱ��Ȧ��ָʾ�Ƹ��º���*******/
void OverCurrent_CoilxLED_State_Update(Board_Info	Board_Running_Info)
{
		static uint8_t OCcnt_indexA,OCcnt_indexB,OCcnt_indexC,OCcnt_indexD;
	
		if(OverCurrent_Err==Board_Running_Info.Coil_A) 
		{
			LEDx_Flashing_Operation(CoilA_LED_Pin,Sequence_FlashingQuick_Repeated,OCcnt_indexA);		//�ж���ȦA������LED_CoilA������˸
			if(++OCcnt_indexA>sizeof(Sequence_FlashingQuick_Repeated)-1) OCcnt_indexA=0;
		}
		else LEDx_OFF(CoilA_LED_Pin);
		
		if(OverCurrent_Err==Board_Running_Info.Coil_B) 
		{
			LEDx_Flashing_Operation(CoilB_LED_Pin,Sequence_FlashingQuick_Repeated,OCcnt_indexB);		//�ж���ȦB������LED_CoilA������˸
			if(++OCcnt_indexB>sizeof(Sequence_FlashingQuick_Repeated)-1) OCcnt_indexB=0;
		}
		else LEDx_OFF(CoilB_LED_Pin);
		
		if(OverCurrent_Err==Board_Running_Info.Coil_C) 
		{
			LEDx_Flashing_Operation(CoilC_LED_Pin,Sequence_FlashingQuick_Repeated,OCcnt_indexC);		//�ж���ȦC������LED_CoilA������˸
			if(++OCcnt_indexC>sizeof(Sequence_FlashingQuick_Repeated)-1) OCcnt_indexC=0;
		}
		else LEDx_OFF(CoilC_LED_Pin);
		
		if(OverCurrent_Err==Board_Running_Info.Coil_D) 
		{
			LEDx_Flashing_Operation(CoilD_LED_Pin,Sequence_FlashingQuick_Repeated,OCcnt_indexD);		//�ж���ȦD������LED_CoilA������˸
			if(++OCcnt_indexD>sizeof(Sequence_FlashingQuick_Repeated)-1) OCcnt_indexD=0;
		}
		else LEDx_OFF(CoilD_LED_Pin);
}


/********��·ʱ��Ȧ��ָʾ�Ƹ��º���*******/
void OpenCircuit_CoilxLED_State_Update(Board_Info	Board_Running_Info)
{
		static uint8_t OPcnt_indexA,OPcnt_indexB,OPcnt_indexC,OPcnt_indexD;
	
		if(OpenCircuit_Err==Board_Running_Info.Coil_A) 
		{
			LEDx_Flashing_Operation(CoilA_LED_Pin,Sequence_FlashingQuick_Twice,OPcnt_indexA);		//�ж���ȦA������LED_CoilA������˸����
			if(++OPcnt_indexA>sizeof(Sequence_FlashingQuick_Twice)-1) OPcnt_indexA=0;
		}
		else LEDx_OFF(CoilA_LED_Pin);
		
		if(OpenCircuit_Err==Board_Running_Info.Coil_B) 
		{
			LEDx_Flashing_Operation(CoilB_LED_Pin,Sequence_FlashingQuick_Twice,OPcnt_indexB);		//�ж���ȦB������LED_CoilB������˸����
			if(++OPcnt_indexB>sizeof(Sequence_FlashingQuick_Twice)-1) OPcnt_indexB=0;
		}
		else LEDx_OFF(CoilB_LED_Pin);
		
		if(OpenCircuit_Err==Board_Running_Info.Coil_C) 
		{
			LEDx_Flashing_Operation(CoilC_LED_Pin,Sequence_FlashingQuick_Twice,OPcnt_indexC);		//�ж���ȦC������LED_CoilC������˸����
			if(++OPcnt_indexC>sizeof(Sequence_FlashingQuick_Twice)-1) OPcnt_indexC=0;
		}
		else LEDx_OFF(CoilC_LED_Pin);
		
		if(OpenCircuit_Err==Board_Running_Info.Coil_D) 
		{
			LEDx_Flashing_Operation(CoilD_LED_Pin,Sequence_FlashingQuick_Twice,OPcnt_indexD);		//�ж���ȦD������LED_CoilD������˸����
			if(++OPcnt_indexD>sizeof(Sequence_FlashingQuick_Twice)-1) OPcnt_indexD=0;
		}
		else LEDx_OFF(CoilD_LED_Pin);
}

/****������ȫ��LED��״̬���º�����ָ����������Ϊ250ms !!********/
void BoardLED_State_Update(uint32_t Board_State,Board_Info	Board_Running_Info)
{
	switch(Board_State)
	{
		case Initial:
					break;
		
		case Standby:
								LEDx_Flashing_Operation(BoardState_LED_Pin,Sequence_FlashingSlow_Repeated,led_BoardState);//board state LED ������˸
								if(++led_BoardState>sizeof(Sequence_FlashingSlow_Repeated)-1) led_BoardState=0; 
								LEDx_OFF(CoilA_LED_Pin);																																	//�ر������ĵ�
								LEDx_OFF(CoilB_LED_Pin);
								LEDx_OFF(CoilC_LED_Pin);
								LEDx_OFF(CoilD_LED_Pin);	
					break;
		case Running:
								LEDx_Flashing_Operation(BoardState_LED_Pin,Sequence_FlashingQuick_Repeated,led_BoardState);//board state LED ������˸
								if(++led_BoardState>sizeof(Sequence_FlashingQuick_Repeated)-1) led_BoardState=0; 
								Running_CoilxLED_State_Update(Board_Running_Info);																				//CoilLED��
					break;
		
		case OverCurrent_Err:	
								LEDx_OFF(BoardState_LED_Pin);																															//board state LED �ر�
								OverCurrent_CoilxLED_State_Update(Board_Running_Info);																		 //CoilLED������˸
					break;
		
		case OpenCircuit_Err:
								LEDx_OFF(BoardState_LED_Pin);																															//board state LED �ر�
								OpenCircuit_CoilxLED_State_Update(Board_Running_Info);																		 //CoilxLED������˸����
					break;
		case CanBus_Comm_Err:
								LEDx_Flashing_Operation(BoardState_LED_Pin,Sequence_FlashingQuick_Twice,led_BoardState);//board state LED ������˸����
								if(++led_BoardState>sizeof(Sequence_FlashingQuick_Twice)-1) led_BoardState=0; 
								LEDx_OFF(CoilA_LED_Pin);																																	//�ر������ĵ�
								LEDx_OFF(CoilB_LED_Pin);
								LEDx_OFF(CoilC_LED_Pin);
								LEDx_OFF(CoilD_LED_Pin);

					break;
		case ADC_Trans_Err_Soft:
								LEDx_Flashing_Operation(BoardState_LED_Pin,Sequence_FlashingQuick_Thrice,led_BoardState);//board state LED ������˸����
								if(++led_BoardState>sizeof(Sequence_FlashingQuick_Thrice)-1) led_BoardState=0; 
								LEDx_OFF(CoilA_LED_Pin);																																	//�ر������ĵ�
								LEDx_OFF(CoilB_LED_Pin);
								LEDx_OFF(CoilC_LED_Pin);
								LEDx_OFF(CoilD_LED_Pin);
					break;
		case ADC_Trans_Err_Hard:
								LEDx_OFF(BoardState_LED_Pin);		//���еƹر�
								LEDx_OFF(CoilA_LED_Pin);																																	
								LEDx_OFF(CoilB_LED_Pin);
								LEDx_OFF(CoilC_LED_Pin);
								LEDx_OFF(CoilD_LED_Pin);
					break;
		default:
					break;
	}

}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
