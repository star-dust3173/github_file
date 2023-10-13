#include "pid.h"

void PID_param_init(_PID* pid_location,_PID* pid_speed)
{
    /* 位置相关初始化参数 */
            
    pid_location->output_val = 0.0;
    pid_location->err = 0.0;
    pid_location->err_old = 0.0;


    pid_location->P = 5;
    pid_location->I = 0;
    pid_location->D = 0;

    /* 速度相关初始化参数 */
                 
    pid_speed->output_val=0.0;
    pid_speed->err=0.0;
    pid_speed->err_old=0.0;

    pid_speed->P = 10.0;
    pid_speed->I = 0.0;
    pid_speed->D = 0.0;
}

float PID_Cal_Limt(_PID *PID, float limit, float get, float set)//PID
{
	PID->err = set - get;
	PID->err_err = PID->err - PID->err_old;
	
	PID->P_OUT  = PID->P * PID->err;
	PID->I_OUT += PID->I * PID->err;
	PID->D_OUT  = PID->D * PID->err_err;
	
	PID->I_OUT = (PID->I_OUT > PID->I_LIMIT)?(PID->I_LIMIT):((PID->I_OUT < -PID->I_LIMIT)?(-PID->I_LIMIT):(PID->I_OUT));
	
	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;
	
	PID->OUT = (PID->OUT > PID->OUT_LIMIT)?(PID->OUT_LIMIT):((PID->OUT < -PID->OUT_LIMIT)?(-PID->OUT_LIMIT):(PID->OUT));
	
	if(ABS(PID->err) <= ABS(limit))
	{
	  PID->I_OUT=0;
	  PID->OUT=0;
	}
	
	PID->err_old = PID->err;
	
	return PID->OUT;
}





void Motor_Auto_Run(void)
{
	float motor_err=0; 						//死区控制参数
	uint8_t mode=0; 								//mode0-a；mode1-b
	float angle_a=360;
	float angle_b=90;	//a-角度；b-角度
	motor_err = 50;
	
	float abs_err=0;
	static float abs_err_old=0; 
	
	motor.ANGLE.POS_ABS = motor_data.real_angle;
//	Motor_Angle_Cal(360);//得到绝对角度
	//a
	if(mode == 0)
	{
		
	motor.ANGLE.POS_GAOL = angle_a;
		
	PID_Cal_Limt(&motor.PID_ANGLE, motor_err, motor.ANGLE.POS_ABS,motor.ANGLE.POS_GAOL);

	abs_err = motor.ANGLE.POS_ABS - abs_err_old;
	
	PID_Cal_Limt( &motor.PID_SPEED, 10, abs_err, motor.PID_ANGLE.OUT);
	
	abs_err_old = motor.ANGLE.POS_ABS;
	CAN_Tx_Message((int16_t)(motor.PID_SPEED.OUT));
	}
	//b
	else if (mode == 1)
	{
		
	motor.ANGLE.POS_GAOL = angle_b  ;
	PID_Cal_Limt(&motor.PID_ANGLE, motor_err, motor.ANGLE.POS_ABS,motor.ANGLE.POS_GAOL);
	
	abs_err = motor.ANGLE.POS_ABS - abs_err_old;
		
	PID_Cal_Limt( &motor.PID_SPEED, 10, abs_err, motor.PID_ANGLE.OUT);
		
	abs_err_old = motor.ANGLE.POS_ABS;
	CAN_Tx_Message((int16_t)(motor.PID_SPEED.OUT));
		
	}
	

}


