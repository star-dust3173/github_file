#include "pid2.h"


/**
  * @brief  PID������ʼ��
  * @note   ��
  * @retval ��
  */
void PID_param_Init(PID* pid_location,PID* pid_speed)
{
    /* λ����س�ʼ������ */
    pid_location->target_val = TOTAL_RESOLUTION;              
    pid_location->output_val = 0.0;
    pid_location->err = 0.0;
    pid_location->err_last = 0.0;
    pid_location->integral = 0.0;

    pid_location->Kp = 1;
    pid_location->Ki = 0.1;
    pid_location->Kd = 0;

    /* �ٶ���س�ʼ������ */
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
  * @brief  λ��PID�㷨ʵ��
  * @param  actual_val:ʵ��ֵ
  * @note   ��
  * @retval ͨ��PID���������
  */

float location_pid_realize(PID *pid, float actual_val)
{
    /*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err = pid->target_val - actual_val;

    /* �趨�ջ����� */
    if((pid->err >= -LOC_DEAD_ZONE) && (pid->err <= LOC_DEAD_ZONE))
    {
        pid->err = 0;
        pid->integral = 0;
        pid->err_last = 0;
    }

    /*��������ַ��룬ƫ��ϴ�ʱȥ����������*/
    if(pid->err > -LOC_INTEGRAL_START_ERR && pid->err < LOC_INTEGRAL_START_ERR)
    {
        pid->integral += pid->err;  
        /*���ַ�Χ�޶�����ֹ���ֱ���*/
        if(pid->integral > LOC_INTEGRAL_MAX_VAL)
        {
            pid->integral = LOC_INTEGRAL_MAX_VAL;
        }
        else if(pid->integral < -LOC_INTEGRAL_MAX_VAL)
        {
            pid->integral = -LOC_INTEGRAL_MAX_VAL;
        }
    }   

    /*PID�㷨ʵ��*/
    pid->output_val = pid->Kp * pid->err +
                      pid->Ki * pid->integral +
                      pid->Kd * (pid->err - pid->err_last);

    /*����*/
    pid->err_last = pid->err;

    /*���ص�ǰʵ��ֵ*/
    return pid->output_val;
}

/**
  * @brief  �ٶ�PID�㷨ʵ��
  * @param  actual_val:ʵ��ֵ
  * @note   ��
  * @retval ͨ��PID���������
  */

float speed_pid_realize(PID *pid, float actual_val)
{
    /*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err = pid->target_val - actual_val;

    /* �趨�ջ����� */
    if( (pid->err>-SPE_DEAD_ZONE) && (pid->err<SPE_DEAD_ZONE ) )
    {
        pid->err = 0;
        pid->integral = 0;
        pid->err_last = 0;
    }

    /*��������ַ��룬ƫ��ϴ�ʱȥ����������*/
    if(pid->err > -SPE_INTEGRAL_START_ERR && pid->err < SPE_INTEGRAL_START_ERR)
    {
        pid->integral += pid->err;  
        /*���ַ�Χ�޶�����ֹ���ֱ���*/
        if(pid->integral > SPE_INTEGRAL_MAX_VAL)
        {
            pid->integral = SPE_INTEGRAL_MAX_VAL;
        }
        else if(pid->integral < -SPE_INTEGRAL_MAX_VAL)
        {
            pid->integral = -SPE_INTEGRAL_MAX_VAL;
        }
    }   

    /*PID�㷨ʵ��*/
    pid->output_val = pid->Kp * pid->err +
                      pid->Ki * pid->integral +
                      pid->Kd *(pid->err - pid->err_last);

    /*����*/
    pid->err_last = pid->err;

    /*���ص�ǰʵ��ֵ*/
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
