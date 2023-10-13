#ifndef __M_PID
#define __M_PID

#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

typedef struct
{
	    uint8_t mode;

    //PID 三参数
    float Kp,Ki,Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出
		float deadband;	//死区
		float i_split;	//积分分离变量
	
    float ref;			//目标速度
    float fdb;			//当前速度
		float lastfdb;	//上一速度

    float out;			//速度输出
    float Pout;			//p参数输出(内部)
    float Iout;			//i参数输出(内部)
		float last_Iout;//i参数上一输出(内部)
    float Dout;			//d参数输出(内部)
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;



void PID_Init(PidTypeDef *pid,  const float PID[5]);
float PID_Calc(PidTypeDef *pid, float ref, float set);
void PID_clear(PidTypeDef *pid);

#endif


