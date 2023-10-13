#ifndef __M_PID
#define __M_PID

#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

typedef struct
{
	    uint8_t mode;

    //PID ������
    float Kp,Ki,Kd;

    float max_out;  //������
    float max_iout; //���������
		float deadband;	//����
		float i_split;	//���ַ������
	
    float ref;			//Ŀ���ٶ�
    float fdb;			//��ǰ�ٶ�
		float lastfdb;	//��һ�ٶ�

    float out;			//�ٶ����
    float Pout;			//p�������(�ڲ�)
    float Iout;			//i�������(�ڲ�)
		float last_Iout;//i������һ���(�ڲ�)
    float Dout;			//d�������(�ڲ�)
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�

} PidTypeDef;



void PID_Init(PidTypeDef *pid,  const float PID[5]);
float PID_Calc(PidTypeDef *pid, float ref, float set);
void PID_clear(PidTypeDef *pid);

#endif


