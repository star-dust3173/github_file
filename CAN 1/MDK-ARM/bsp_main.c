#include "bsp_main.h"

extern _motor_data motor_data;
extern motor_t motor1;

void main_init(motor_t* motor1)
{
	PID_param_init(&motor1->pid_location, &motor1->pid_speed);
	motor1->actual_angle = 0.0f;
	motor1->actual_speed = 0.0f;
}

//void motor_pid()
//{
//	location_pid_realize(&motor1.pid_location, motor1.actual_angle);
//	speed_pid_realize(&motor1.pid_speed, motor1.actual_speed);
//}





