#ifndef __bsp_main
#define	__bsp_main

#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

#include "M_CAN.h"
#include "pid2.h"

typedef struct{
	float actual_speed;
	float actual_angle;
	float set_currunt;
	
	PID pid_location;
	PID pid_speed;
	
}motor_t;


void main_init(motor_t* motor1);
//void motor_pid();

#endif


