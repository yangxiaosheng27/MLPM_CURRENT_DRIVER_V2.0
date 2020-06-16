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

uint8_t Board_Address;		//板子的地址信息
uint8_t Board_Axis;				//板子属于哪个轴	
uint8_t Board_Number;			//板子的序号

uint8_t led_BoardState;									//用于控制板子状态LED的计数变量
uint32_t Board_State = Initial;					//板子当前的运行状态
Board_Info Board_Running_Info;					//用于监控板上关键器件的运行信息
uint32_t CANBus_DataRecieve_TimeCNT;		//用于统计can总线没有收到数据的时间计数变量
/* USER CODE END 0 */
static uint8_t Sequence_FlashingQuick_Twice[7]={1,0,1,0,0,0,0};						//LED快速闪烁两次的赋值队列 didi~didi~didi~
static uint8_t Sequence_FlashingQuick_Thrice[9]={1,0,1,0,1,0,0,0,0};			//LED快速闪烁三次的赋值队列	dididi~dididi~dididi~
static uint8_t Sequence_FlashingSlow_Repeated[6]={1,1,1,0,0,0};						//LED慢速闪烁的赋值队列	di~di~di~	
static uint8_t Sequence_FlashingQuick_Repeated[4]={1,0,1,0};							//LED快速闪烁的赋值队列	di-di-di-	
/* USER CODE BEGIN 1 */

/*****初始化--关闭全部LED*****/
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
/****板子初始化程序：完成板子的地址获取，完成ADC偏置电流获取**********
*****Note:该程序必须使用延时，在延时期间由中断完成ADC偏置电流的矫正和检测
******/
void user_Board_Initial(void)
{
	uint8_t Address=0;
	LED_State_Init();													//关闭所有LED
	Board_State	=	Initial;									//板子状态为初始化状态
	Address	=	Read_AddressBit();						//ON代表1，OFF代表0
	Board_Axis		=	Address&0x01;						//00 0001	最低位是板子的轴编号：0--X轴		1--Y轴
	Board_Number	=	Address>>1&0x1F;				//11 1110	高五位是板子的序号：0~9
	
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
	
	HAL_Delay(100);														//延时100ms，以足够保证板子上的电源全部稳定下来
	
	Get_ADS8674_BiasValue();									//ADC偏置值获取-板子的状态和ADC状态确认
		
	Board_Running_Info.CANBus 	 = 	Standby;
	Board_Running_Info.Coil_A 	 = 	Standby;
	Board_Running_Info.Coil_B 	 = 	Standby;
	Board_Running_Info.Coil_C 	 = 	Standby;
	Board_Running_Info.Coil_D 	 = 	Standby;
}
/**********
//ADC软件检测函数，完成ADC偏置值校验和检测任务
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
//ADC硬件报错检测函数，完成ADC硬件报错检测任务
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
//完成线圈过流检测任务
******/
void Board_Coil_OverCurrent_DEC() 		
{
	/***********过流检测**********/
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
	/*******END 过流检测**********/
}
/**********
//线圈开路检测函数，处理频率 5Hz
******/
void Board_Coil_OpenOpenCircuit_DEC(void)		
{
	/*线圈开路检测原理：板子接收到线圈电流信号，在running状态下，定时检测线圈电流是否为0或者接近0，
										 长时间检测到线圈输出为0，则证明电流环控制无效，线圈开路*/
	static uint8_t OP_TimeCnt_A,OP_TimeCnt_B,OP_TimeCnt_C,OP_TimeCnt_D;							//检测时间计数
	if(Board_State==Running)
	{
/*******************************************************************************************/	
		if(Coil_Enable==Board_Running_Info.Coil_A) //接收到了A线圈的电流信号，A线圈电流环已经使能
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
		if(Coil_Enable==Board_Running_Info.Coil_B) //接收到了B线圈的电流信号，B线圈电流环已经使能
		{
			if(abs(Current_B.Feedback_Value)<OpenCircuit_MeasurementValue)		//期望电流与实际电流相差500mA:要注意，这里可能会存在bug，给定电流在高频跳变的时候可能会误算
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
		if(Coil_Enable==Board_Running_Info.Coil_C) //接收到了C线圈的电流信号，C线圈电流环已经使能
		{
			if(abs(Current_C.Feedback_Value)<OpenCircuit_MeasurementValue)		//期望电流与实际电流相差500mA:要注意，这里可能会存在bug，给定电流在高频跳变的时候可能会误算
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
		if(Coil_Enable==Board_Running_Info.Coil_D) //接收到了D线圈的电流信号，D线圈电流环已经使能
		{
			if(abs(Current_D.Feedback_Value)<OpenCircuit_MeasurementValue)		//期望电流与实际电流相差500mA:要注意，这里可能会存在bug，给定电流在高频跳变的时候可能会误算
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
//CAN总线信号查错函数，结算出来的数据超过规定的最大电流，板子报错，如果没错就给驱动器指定电流信号,并且改变电流环控制参数
******/
void Board_CanBus_DEC(int Recieve_CurrentA,int Recieve_CurrentB,int Recieve_CurrentC,int Recieve_CurrentD)
{
	/**********CoilA************/
	if(0!=Recieve_CurrentA)	//接收到的电流指令不为0
	{
		Board_Running_Info.Coil_A=Coil_Enable;
	}
	if(abs(Recieve_CurrentA)>CANBUS_RECIEVE_MAX_CURRENT)//接收到的电流指令大于指定的最大值
	{
		Board_State=CanBus_Comm_Err;
		Board_Running_Info.Coil_A=Coil_Disable;
	}
	else 
	{
		Current_A.Expect_Value = Recieve_CurrentA;
		Current_A.PID.Ki = Initial_Ki+fabs(Recieve_CurrentA*0.000006f);//积分参数从0-15A线性增长
	}
	/**********CoilB************/
	if(0!=Recieve_CurrentB)	//接收到的电流指令不为0
	{
		Board_Running_Info.Coil_B=Coil_Enable;
	}
	if(abs(Recieve_CurrentB)>CANBUS_RECIEVE_MAX_CURRENT)//接收到的电流指令大于指定的最大值
	{
		Board_State=CanBus_Comm_Err;
		Board_Running_Info.Coil_B=Coil_Disable;
	}
	else 
	{
		Current_B.Expect_Value=Recieve_CurrentB;
		Current_B.PID.Ki = Initial_Ki+fabs(Recieve_CurrentB*0.000006f);//积分参数从0-15A线性增长
	}
	/**********CoilC************/	
		if(0!=Recieve_CurrentC)	//接收到的电流指令不为0
	{
		Board_Running_Info.Coil_C=Coil_Enable;
	}
	if(abs(Recieve_CurrentC)>CANBUS_RECIEVE_MAX_CURRENT)//接收到的电流指令大于指定的最大值
	{
		Board_State=CanBus_Comm_Err;
		Board_Running_Info.Coil_C=Coil_Disable;
	}
	else 
	{
		Current_C.Expect_Value=Recieve_CurrentC;
		Current_C.PID.Ki = Initial_Ki+fabs(Recieve_CurrentC*0.000006f);//积分参数从0-15A线性增长
	}
	
	/**********CoilD************/	
		if(0!=Recieve_CurrentD)	//接收到的电流指令不为0
	{
		Board_Running_Info.Coil_D=Coil_Enable;
	}
	if(abs(Recieve_CurrentD)>CANBUS_RECIEVE_MAX_CURRENT)//接收到的电流指令大于指定的最大值
	{
		Board_State=CanBus_Comm_Err;
		Board_Running_Info.Coil_D=Coil_Disable;
	}
	else 
	{
		Current_D.Expect_Value=Recieve_CurrentD;
		Current_D.PID.Ki = Initial_Ki+fabs(Recieve_CurrentD*0.000006f);//积分参数从0-15A线性增长
	}
	
}
/**********
//CAN总线信号检测函数，超过一定时长没有接收到CAN总线的数据之后，板子恢复到Standby状态
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
/*****LED闪烁控制函数*******/
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
/********正常运行时线圈的指示灯更新函数*******/
void Running_CoilxLED_State_Update(Board_Info	Board_Running_Info)
{
		if(Coil_Enable==Board_Running_Info.Coil_A) LEDx_ON(CoilA_LED_Pin);												//接收到线圈A的电流命令
		else LEDx_OFF(CoilA_LED_Pin);
		if(Coil_Enable==Board_Running_Info.Coil_B) LEDx_ON(CoilB_LED_Pin);												//接收到线圈B的电流命令
		else LEDx_OFF(CoilB_LED_Pin);
		if(Coil_Enable==Board_Running_Info.Coil_C) LEDx_ON(CoilC_LED_Pin);												//接收到线圈C的电流命令
		else LEDx_OFF(CoilC_LED_Pin);
		if(Coil_Enable==Board_Running_Info.Coil_D) LEDx_ON(CoilD_LED_Pin);												//接收到线圈D的电流命令
		else LEDx_OFF(CoilD_LED_Pin);
}
/********过流时线圈的指示灯更新函数*******/
void OverCurrent_CoilxLED_State_Update(Board_Info	Board_Running_Info)
{
		static uint8_t OCcnt_indexA,OCcnt_indexB,OCcnt_indexC,OCcnt_indexD;
	
		if(OverCurrent_Err==Board_Running_Info.Coil_A) 
		{
			LEDx_Flashing_Operation(CoilA_LED_Pin,Sequence_FlashingQuick_Repeated,OCcnt_indexA);		//判断线圈A过流，LED_CoilA快速闪烁
			if(++OCcnt_indexA>sizeof(Sequence_FlashingQuick_Repeated)-1) OCcnt_indexA=0;
		}
		else LEDx_OFF(CoilA_LED_Pin);
		
		if(OverCurrent_Err==Board_Running_Info.Coil_B) 
		{
			LEDx_Flashing_Operation(CoilB_LED_Pin,Sequence_FlashingQuick_Repeated,OCcnt_indexB);		//判断线圈B过流，LED_CoilA快速闪烁
			if(++OCcnt_indexB>sizeof(Sequence_FlashingQuick_Repeated)-1) OCcnt_indexB=0;
		}
		else LEDx_OFF(CoilB_LED_Pin);
		
		if(OverCurrent_Err==Board_Running_Info.Coil_C) 
		{
			LEDx_Flashing_Operation(CoilC_LED_Pin,Sequence_FlashingQuick_Repeated,OCcnt_indexC);		//判断线圈C过流，LED_CoilA快速闪烁
			if(++OCcnt_indexC>sizeof(Sequence_FlashingQuick_Repeated)-1) OCcnt_indexC=0;
		}
		else LEDx_OFF(CoilC_LED_Pin);
		
		if(OverCurrent_Err==Board_Running_Info.Coil_D) 
		{
			LEDx_Flashing_Operation(CoilD_LED_Pin,Sequence_FlashingQuick_Repeated,OCcnt_indexD);		//判断线圈D过流，LED_CoilA快速闪烁
			if(++OCcnt_indexD>sizeof(Sequence_FlashingQuick_Repeated)-1) OCcnt_indexD=0;
		}
		else LEDx_OFF(CoilD_LED_Pin);
}


/********开路时线圈的指示灯更新函数*******/
void OpenCircuit_CoilxLED_State_Update(Board_Info	Board_Running_Info)
{
		static uint8_t OPcnt_indexA,OPcnt_indexB,OPcnt_indexC,OPcnt_indexD;
	
		if(OpenCircuit_Err==Board_Running_Info.Coil_A) 
		{
			LEDx_Flashing_Operation(CoilA_LED_Pin,Sequence_FlashingQuick_Twice,OPcnt_indexA);		//判断线圈A过流，LED_CoilA快速闪烁两次
			if(++OPcnt_indexA>sizeof(Sequence_FlashingQuick_Twice)-1) OPcnt_indexA=0;
		}
		else LEDx_OFF(CoilA_LED_Pin);
		
		if(OpenCircuit_Err==Board_Running_Info.Coil_B) 
		{
			LEDx_Flashing_Operation(CoilB_LED_Pin,Sequence_FlashingQuick_Twice,OPcnt_indexB);		//判断线圈B过流，LED_CoilB快速闪烁两次
			if(++OPcnt_indexB>sizeof(Sequence_FlashingQuick_Twice)-1) OPcnt_indexB=0;
		}
		else LEDx_OFF(CoilB_LED_Pin);
		
		if(OpenCircuit_Err==Board_Running_Info.Coil_C) 
		{
			LEDx_Flashing_Operation(CoilC_LED_Pin,Sequence_FlashingQuick_Twice,OPcnt_indexC);		//判断线圈C过流，LED_CoilC快速闪烁两次
			if(++OPcnt_indexC>sizeof(Sequence_FlashingQuick_Twice)-1) OPcnt_indexC=0;
		}
		else LEDx_OFF(CoilC_LED_Pin);
		
		if(OpenCircuit_Err==Board_Running_Info.Coil_D) 
		{
			LEDx_Flashing_Operation(CoilD_LED_Pin,Sequence_FlashingQuick_Twice,OPcnt_indexD);		//判断线圈D过流，LED_CoilD快速闪烁两次
			if(++OPcnt_indexD>sizeof(Sequence_FlashingQuick_Twice)-1) OPcnt_indexD=0;
		}
		else LEDx_OFF(CoilD_LED_Pin);
}

/****板子上全部LED的状态更新函数，指定运行周期为250ms !!********/
void BoardLED_State_Update(uint32_t Board_State,Board_Info	Board_Running_Info)
{
	switch(Board_State)
	{
		case Initial:
					break;
		
		case Standby:
								LEDx_Flashing_Operation(BoardState_LED_Pin,Sequence_FlashingSlow_Repeated,led_BoardState);//board state LED 慢速闪烁
								if(++led_BoardState>sizeof(Sequence_FlashingSlow_Repeated)-1) led_BoardState=0; 
								LEDx_OFF(CoilA_LED_Pin);																																	//关闭其他的灯
								LEDx_OFF(CoilB_LED_Pin);
								LEDx_OFF(CoilC_LED_Pin);
								LEDx_OFF(CoilD_LED_Pin);	
					break;
		case Running:
								LEDx_Flashing_Operation(BoardState_LED_Pin,Sequence_FlashingQuick_Repeated,led_BoardState);//board state LED 快速闪烁
								if(++led_BoardState>sizeof(Sequence_FlashingQuick_Repeated)-1) led_BoardState=0; 
								Running_CoilxLED_State_Update(Board_Running_Info);																				//CoilLED打开
					break;
		
		case OverCurrent_Err:	
								LEDx_OFF(BoardState_LED_Pin);																															//board state LED 关闭
								OverCurrent_CoilxLED_State_Update(Board_Running_Info);																		 //CoilLED快速闪烁
					break;
		
		case OpenCircuit_Err:
								LEDx_OFF(BoardState_LED_Pin);																															//board state LED 关闭
								OpenCircuit_CoilxLED_State_Update(Board_Running_Info);																		 //CoilxLED快速闪烁两次
					break;
		case CanBus_Comm_Err:
								LEDx_Flashing_Operation(BoardState_LED_Pin,Sequence_FlashingQuick_Twice,led_BoardState);//board state LED 快速闪烁两次
								if(++led_BoardState>sizeof(Sequence_FlashingQuick_Twice)-1) led_BoardState=0; 
								LEDx_OFF(CoilA_LED_Pin);																																	//关闭其他的灯
								LEDx_OFF(CoilB_LED_Pin);
								LEDx_OFF(CoilC_LED_Pin);
								LEDx_OFF(CoilD_LED_Pin);

					break;
		case ADC_Trans_Err_Soft:
								LEDx_Flashing_Operation(BoardState_LED_Pin,Sequence_FlashingQuick_Thrice,led_BoardState);//board state LED 快速闪烁三次
								if(++led_BoardState>sizeof(Sequence_FlashingQuick_Thrice)-1) led_BoardState=0; 
								LEDx_OFF(CoilA_LED_Pin);																																	//关闭其他的灯
								LEDx_OFF(CoilB_LED_Pin);
								LEDx_OFF(CoilC_LED_Pin);
								LEDx_OFF(CoilD_LED_Pin);
					break;
		case ADC_Trans_Err_Hard:
								LEDx_OFF(BoardState_LED_Pin);		//所有灯关闭
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
