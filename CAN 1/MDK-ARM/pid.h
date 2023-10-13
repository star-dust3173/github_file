#ifndef __pid
#define __pid

#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"
#include "M_CAN.h"

extern MOTOR_TypeDef motor;

extern _motor_data motor_data;

void PID_param_init(_PID* pid_location,_PID* pid_speed);
	
float PID_Cal_Limt(_PID *PID, float limit, float get, float set);//PIDËÀÇøÐÞ¸Ä
void Motor_Auto_Run(void);
	
#endif


