#include "Test.h"
#include "Motor.h"
#include "PID.h"
#include "FOC.h"

extern Encoder_Struct encoder_str;     //编码器结构体
extern SVPWM_Struct svpwm_str;         //SVPWM结构体
extern PID pid_m1;                     //PID参数结构体
extern Motor_Test Motor;               //电机测试变量

/************
电流环带宽测试：
	如果不调整正确的带宽值，就会出现电流环跟随的幅值不变
	或者衰减太厉害

	电流环的Kp，Ki参数为：
		带宽=300*(2*PI) ,主要修改300那个值
		Kp=相电感*带宽
		Ki=相电阻*带宽*Ts  (Ts为电流环周期)
		
	原理如下：
	先设置好电流环的Kp，Ki，然后给电流环注入一个高频正弦信号
	赋值Iq为0，只修改Id目标值，使得电机不转动，正常情况下在
	电流环里面写入  Cal_IWidth()  函数，把其他的全部注释掉
	观察   Id_aim=Id_current*根号2，
	因为如果带宽(Motor.Iwidth)正确，实际电流值会衰减到设定值的 根号2分之1
	如果带宽(Motor.Iwidth)设置过大，则会衰减更大，使得Id_current更小，
	反之Id_current更大

使用说明：在电流环模式里面写入  Cal_IWidth()  函数，把其他的全部注释掉
**************/

void Cal_IWidth(void)
{
	float theta;
	theta += Motor.Iwidth*2*PI/20000;
	theta = Angle_Limit(theta,2*PI);
	
	allflag.Encoder_flag = 3;
	pid_m1.Iq_aim = 0.0f;
	pid_m1.Id_aim = encoder_str.Encoder_Mode1_Angle*arm_sin_f32(theta);
	PID_I_Control(&pid_m1);
	SVPWM(pid_m1.Uq,pid_m1.Ud,encoder_str.Return_Angle,&svpwm_str);
}	

/************
弱磁补偿
**************/





