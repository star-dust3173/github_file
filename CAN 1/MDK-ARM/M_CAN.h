#ifndef __M_CAN
#define __M_CAN

#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

typedef struct
{
	int16_t angle_value;
	int16_t speed_rpm;
	int16_t real_current;
	int16_t temperature;
	int16_t real_angle;
}_motor_data;

typedef struct
{
	float POS_GAOL;//目标位置
	float POS_ABS;//绝对位置0
	float POS_OFFSET;
	float eer;
	float eer_eer;
}ANGLE_TypeDef;

//typedef struct
//{
////    float target_val;   //目标值
//    float err;          //偏差值
//		float err_err;
//    float err_old;     //上一个偏差值
//    float P,I,D;     		//比例、积分、微分系数
//		float P_OUT,I_OUT,D_OUT;
//		float OUT;
//		float I_LIMIT;
//		float OUT_LIMIT;

//    float output_val;   //输出值
//}_PID;

typedef struct
{
    float target_val;   //目标值
    float err;          //偏差值
    float err_last;     //上一个偏差值
    float Kp,Ki,Kd;     //比例、积分、微分系数
    float integral;     //积分值
    float output_val;   //输出值
}PID;

//typedef struct
//{
//	ANGLE_TypeDef ANGLE;
//	_PID   PID_SPEED;
//	_PID   PID_ANGLE;
//	
//}MOTOR_TypeDef;


typedef struct
{
	ANGLE_TypeDef ANGLE;
	PID   PID_SPEED;
	PID   PID_ANGLE;
	
}MOTOR_TypeDef;

//int16_t speed=400;
extern MOTOR_TypeDef motor;
extern _motor_data motor_data;

uint8_t Bsp_canInit(void);
void CAN_Tx_Message(uint16_t cur);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void Motor_Angle_Cal(float T);
float ABS(float a);

#endif

