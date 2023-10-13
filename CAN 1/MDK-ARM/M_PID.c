#include "M_PID.h"

PidTypeDef motor_pid;
const float PID1[6]={12.0f,0.1f,10.0f,800.0f,10.0f,10.0f};		//设定pid参数(kp,ki,kd,最大输出,最大积分输出,死线)

//pid初始化
void PID_Init(PidTypeDef *pid,  const float PID[6])
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = PID[3];
    pid->max_iout = PID[4]; 
		pid->deadband= PID[5];
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}


//最大值比较函数
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


		
//pid计算(pid结构体，目标速度，当前速度)
float PID_Calc(PidTypeDef *pid,  float ref,float fdb)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->ref = ref;
    pid->fdb = fdb;
    pid->error[0] = ref - fdb;
		
	if(pid->error[0] > -pid->deadband && pid->error[0] < pid->deadband) //死区
	pid->error[0]=0;

		
 
	pid->Pout = pid->Kp * pid->error[0];
	pid->last_Iout=pid->Iout;
	
	//减少比例输出时，i的累加
	if(pid->Pout>=pid->max_out)
			pid->Pout=pid->max_out;
	if(pid->Pout<=-pid->max_out)
		pid->Pout=-pid->max_out;

	pid->Iout += pid->Ki * pid->error[0];
	LimitMax(pid->Iout, pid->max_iout);
		
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Dbuf[0];
	pid->out = pid->Pout + pid->Iout + pid->Dout;
		
	if(pid->error[0] > -pid->i_split && pid->error[0] < pid->i_split) //死区执行积分分离
	{
		pid->out = pid->Pout + pid->Dout;
		pid->Iout=pid->last_Iout;
	}
	
	else
		pid->out = pid->Pout + pid->Iout + pid->Dout;
	
	LimitMax(pid->out, pid->max_out);
	
    return pid->out;
}

//pid复位
void PID_clear(PidTypeDef *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->ref = 0.0f;
}
