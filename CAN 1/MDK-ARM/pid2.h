#ifndef __pid2
#define __pid2

#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"
#include "M_CAN.h"




/*定义位置PID与速度PID结构体型的全局变量*/


#define TOTAL_RESOLUTION (360)				//位置目标值

#define LOC_DEAD_ZONE 60 						//位置环死区
#define LOC_INTEGRAL_START_ERR 200 	//积分分离时对应的误差范围
#define LOC_INTEGRAL_MAX_VAL 800   	//积分范围限定，防止积分饱和

#define SPE_DEAD_ZONE 5.0f 					//速度环死区
#define SPE_INTEGRAL_START_ERR 100 	//积分分离时对应的误差范围
#define SPE_INTEGRAL_MAX_VAL 260   	//积分范围限定，防止积分饱和

void PID_param_Init(PID* pid_location,PID* pid_speed);
float location_pid_realize(PID *pid, float actual_val);
float speed_pid_realize(PID *pid, float actual_val);
void Motor_Auto_Run(void);

extern MOTOR_TypeDef motor;
extern _motor_data motor_data;
	extern float err;

#endif


