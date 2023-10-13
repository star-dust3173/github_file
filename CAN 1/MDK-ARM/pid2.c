#include "pid2.h"


/**
  * @brief  PID参数初始化
  * @note   无
  * @retval 无
  */
void PID_param_Init(PID* pid_location,PID* pid_speed)
{
    /* 位置相关初始化参数 */
    pid_location->target_val = TOTAL_RESOLUTION;              
    pid_location->output_val = 0.0;
    pid_location->err = 0.0;
    pid_location->err_last = 0.0;
    pid_location->integral = 0.0;

    pid_location->Kp = 1;
    pid_location->Ki = 0.1;
    pid_location->Kd = 0;

    /* 速度相关初始化参数 */
    pid_speed->target_val=0.0;              
    pid_speed->output_val=0.0;
    pid_speed->err=0.0;
    pid_speed->err_last=0.0;
    pid_speed->integral=0.0;

    pid_speed->Kp = 8.0;
    pid_speed->Ki = 1;
    pid_speed->Kd = 1.0;
		
}

/**
  * @brief  位置PID算法实现
  * @param  actual_val:实际值
  * @note   无
  * @retval 通过PID计算后的输出
  */

float location_pid_realize(PID *pid, float actual_val)
{
    /*计算目标值与实际值的误差*/
    pid->err = pid->target_val - actual_val;

    /* 设定闭环死区 */
    if((pid->err >= -LOC_DEAD_ZONE) && (pid->err <= LOC_DEAD_ZONE))
    {
        pid->err = 0;
        pid->integral = 0;
        pid->err_last = 0;
    }

    /*积分项，积分分离，偏差较大时去掉积分作用*/
    if(pid->err > -LOC_INTEGRAL_START_ERR && pid->err < LOC_INTEGRAL_START_ERR)
    {
        pid->integral += pid->err;  
        /*积分范围限定，防止积分饱和*/
        if(pid->integral > LOC_INTEGRAL_MAX_VAL)
        {
            pid->integral = LOC_INTEGRAL_MAX_VAL;
        }
        else if(pid->integral < -LOC_INTEGRAL_MAX_VAL)
        {
            pid->integral = -LOC_INTEGRAL_MAX_VAL;
        }
    }   

    /*PID算法实现*/
    pid->output_val = pid->Kp * pid->err +
                      pid->Ki * pid->integral +
                      pid->Kd * (pid->err - pid->err_last);

    /*误差传递*/
    pid->err_last = pid->err;

    /*返回当前实际值*/
    return pid->output_val;
}

/**
  * @brief  速度PID算法实现
  * @param  actual_val:实际值
  * @note   无
  * @retval 通过PID计算后的输出
  */

float speed_pid_realize(PID *pid, float actual_val)
{
    /*计算目标值与实际值的误差*/
    pid->err = pid->target_val - actual_val;

    /* 设定闭环死区 */
    if( (pid->err>-SPE_DEAD_ZONE) && (pid->err<SPE_DEAD_ZONE ) )
    {
        pid->err = 0;
        pid->integral = 0;
        pid->err_last = 0;
    }

    /*积分项，积分分离，偏差较大时去掉积分作用*/
    if(pid->err > -SPE_INTEGRAL_START_ERR && pid->err < SPE_INTEGRAL_START_ERR)
    {
        pid->integral += pid->err;  
        /*积分范围限定，防止积分饱和*/
        if(pid->integral > SPE_INTEGRAL_MAX_VAL)
        {
            pid->integral = SPE_INTEGRAL_MAX_VAL;
        }
        else if(pid->integral < -SPE_INTEGRAL_MAX_VAL)
        {
            pid->integral = -SPE_INTEGRAL_MAX_VAL;
        }
    }   

    /*PID算法实现*/
    pid->output_val = pid->Kp * pid->err +
                      pid->Ki * pid->integral +
                      pid->Kd *(pid->err - pid->err_last);

    /*误差传递*/
    pid->err_last = pid->err;

    /*返回当前实际值*/
    return pid->output_val;
}

//	int t = 0;
//	int t0 = 0;
//	int i = 1;
//	extern float err;
	uint8_t sign = 1;
	float h;
	uint8_t mode = 0;			//

void Motor_Auto_Run(void)
{
		
		Motor_Angle_Cal(360);
//		Motor_Angle_Cal(motor.PID_ANGLE.target_val);
		float location;
		float angle;
	location=motor.ANGLE.POS_ABS;
	angle=motor_data.speed_rpm;
		
	
		motor.PID_SPEED.target_val = location_pid_realize(&motor.PID_ANGLE, location);
	
		motor.PID_SPEED.output_val = speed_pid_realize(&motor.PID_SPEED, angle);
	
	if(motor.PID_SPEED.output_val>2000)
		motor.PID_SPEED.output_val=2000;
	else if(motor.PID_SPEED.output_val<-2000)
		motor.PID_SPEED.output_val=-2000;
	
//		while((int)motor.ANGLE.POS_ABS == (int)motor.PID_ANGLE.target_val){
//		CAN_Tx_Message(0);
//		}
	
	CAN_Tx_Message((int16_t)(motor.PID_SPEED.output_val));
		

	
}
