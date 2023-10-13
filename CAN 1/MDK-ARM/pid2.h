#ifndef __pid2
#define __pid2

#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"
#include "M_CAN.h"




/*����λ��PID���ٶ�PID�ṹ���͵�ȫ�ֱ���*/


#define TOTAL_RESOLUTION (360)				//λ��Ŀ��ֵ

#define LOC_DEAD_ZONE 60 						//λ�û�����
#define LOC_INTEGRAL_START_ERR 200 	//���ַ���ʱ��Ӧ����Χ
#define LOC_INTEGRAL_MAX_VAL 800   	//���ַ�Χ�޶�����ֹ���ֱ���

#define SPE_DEAD_ZONE 5.0f 					//�ٶȻ�����
#define SPE_INTEGRAL_START_ERR 100 	//���ַ���ʱ��Ӧ����Χ
#define SPE_INTEGRAL_MAX_VAL 260   	//���ַ�Χ�޶�����ֹ���ֱ���

void PID_param_Init(PID* pid_location,PID* pid_speed);
float location_pid_realize(PID *pid, float actual_val);
float speed_pid_realize(PID *pid, float actual_val);
void Motor_Auto_Run(void);

extern MOTOR_TypeDef motor;
extern _motor_data motor_data;
	extern float err;

#endif


