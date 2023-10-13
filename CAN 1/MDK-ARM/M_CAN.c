#include "M_CAN.h"
#include "math.h"

/*
can通信滤波器初始化
*/
uint8_t Bsp_canInit(void)    
{
	uint8_t status=0;
	CAN_FilterTypeDef canFilter;
	
	
	canFilter.FilterBank=0;    																//筛选器组1
	canFilter.FilterIdHigh=0x0000;
	canFilter.FilterIdLow=0x0000;
	canFilter.FilterMaskIdHigh=0x0000;
	canFilter.FilterMaskIdLow=0x0000;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//掩码模式
	canFilter.FilterActivation=CAN_FILTER_ENABLE;							//开启
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32位模式
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//链接到fifo0
	canFilter.SlaveStartFilterBank=14;													//can筛选组起始编号
	
	status=HAL_CAN_ConfigFilter(&hcan,&canFilter);					//配置过滤器
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	
	return status;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)	//CAN中断
{
	uint8_t             rx_data[8];
	CAN_RxHeaderTypeDef rx_header;
	
	if(hcan->Instance == CAN1)
    {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
    }
	if(rx_header.StdId == 0x209)
	{
		motor_data.angle_value  = rx_data[0] << 8 | rx_data[1];
		motor_data.speed_rpm    = rx_data[2] << 8 | rx_data[3];
		motor_data.real_current = rx_data[4] << 8 | rx_data[5];
		motor_data.temperature = rx_data[6];
			
		motor_data.real_angle = motor_data.angle_value/8192.0f*360.0f;

	}
}

void CAN_Tx_Message(uint16_t cur)
{
		CAN_TxHeaderTypeDef can_Tx;
		uint8_t message[8];
		uint32_t mail_box;
	
	  can_Tx.StdId = 0x2ff;
    can_Tx.IDE = CAN_ID_STD;
    can_Tx.RTR = CAN_RTR_DATA;
    can_Tx.DLC = 0x08;
    can_Tx.TransmitGlobalTime = ENABLE;
		
		message[0]=cur>>8 ;
		message[1]=cur ;
	
		HAL_CAN_AddTxMessage(&hcan, &can_Tx, message, &mail_box);
}

float ABS(float a)
{
	if(a>=0)
		return a;
	else
		return -a;
}

void Motor_Angle_Cal(float T)
{
	float  res1, res2;
	static uint8_t cnt = 1;

	static float pos, pos_old;
	
	if(cnt)
	{
		pos_old = motor_data.real_angle;
		cnt=0;
	}	
	
	pos =motor_data.real_angle;
	motor.ANGLE.eer=pos - pos_old;
	
	if(motor.ANGLE.eer>0) 	
	{
		res1=motor.ANGLE.eer-T;//反转，自减
		res2=motor.ANGLE.eer;
	}
	else
	{
		res1=motor.ANGLE.eer+T;//正转，自加一个周期的角度值（360）
		res2=motor.ANGLE.eer;
	}
	
	if(ABS(res1)<ABS(res2)) //不管正反转，肯定是转的角度小的那个是真的
	{
		motor.ANGLE.eer_eer = res1;
	}
	else
	{
		motor.ANGLE.eer_eer = res2;
	}
	
	motor.ANGLE.POS_ABS += motor.ANGLE.eer_eer;
	pos_old  = pos;
}




