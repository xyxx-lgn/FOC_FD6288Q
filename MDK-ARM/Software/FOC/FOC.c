#include "FOC.h"
#include "Motor.h"


extern AdcValue adcvalue;              //ADC采样变量
extern Encoder_Struct encoder_str;     //编码器结构体
extern SVPWM_Struct svpwm_str;         //SVPWM结构体

/********************
电角度转换：电角度=机械角度*极对数
********************/
float ElectAngle_Turn(float RawAngle,uint8_t pole)
{
	return (RawAngle*pole);
}

/********************
电角度限制：使角度处于0-360
********************/
float ElectAngle_Limit(float Angle)
{
	Angle = 1*ElectAngle_Turn(Angle,7);
	float a =  fmod(Angle,360);    //取余运算用于归一化
	return a>0?a:(a+360);
	/*************
	fmod函数余数的符号和被除数相同。
	例如：当Angle为-PI/2时，fmod(Angle,2*PI)会返回一个负数，因此通过在结果加上2*PI来确保角度值处于0-2PI
	*************/
}



/********************
克拉克变换和帕克变换  
	用于电流环
********************/
void Clark_Park(AdcValue *adcvalue,Encoder_Struct *encoder_str,PID *pid)
{
	/*克拉克变换
		Ialfa = Ia
		Ibeta = (Ia+2Ib)/sqrt(3)
	*/
	pid->Ialfa = adcvalue->Ia;
	pid->Ibeta = (adcvalue->Ia+2*adcvalue->Ib)/sqrt3;
	
	/*帕克变换
		Id = Ialfa*cos+Ibeta*sin
		Iq = -Ialfa*sin+Ibeta*cos
	*/
	pid->Id_current = pid->Ialfa*arm_cos_f32(encoder_str->Return_Angle)+pid->Ibeta*arm_sin_f32(encoder_str->Return_Angle);
	pid->Iq_current = -pid->Ialfa*arm_sin_f32(encoder_str->Return_Angle)+pid->Ibeta*arm_cos_f32(encoder_str->Return_Angle);
}

void SVPWM(float Uq,float Ud,float Angle,SVPWM_Struct *svpwm_str)
{
	float Ualpha=0,Ubeta=0;
	float gain = (float)sqrt3/svpwm_str->Udc*svpwm_str->Ts;
	float Umax_n = 0.577350269f * svpwm_str->Ts;   // =Ts/sqrt3，对应线性圆半径
	//标幺后，再根据载波定标  标幺：sqrt3*Uref/Udc  ,因为SVPWM里面Uref最大值为Udc/sqrt3
	Uq = Uq*gain;
	Ud = Ud*gain;
	
	//圆形限幅
    float mag = sqrtf(Uq*Uq + Ud*Ud);
    if(mag > Umax_n)
    {
        float scale = Umax_n / mag;
        Uq *= scale;
        Ud *= scale;
    }
	
	//帕克逆变换
	Ualpha = Ud*arm_cos_f32(Angle)-Uq*arm_sin_f32(Angle);
	Ubeta  = Ud*arm_sin_f32(Angle)+Uq*arm_cos_f32(Angle);
//	
	
	//通过U1，2，3和ABCN来判断扇区Sector
	svpwm_str->U1 = Ubeta; 								
	svpwm_str->U2 = sqrt3*Ualpha-Ubeta;
	svpwm_str->U3 = -(sqrt3*Ualpha)-Ubeta;	//通过U1，2，3正负来确定ABC值
	
	//确定ABC值{0,1},N值
	if(svpwm_str->U1>0) svpwm_str->A=1;
	else svpwm_str->A=0;
	if(svpwm_str->U2>0) svpwm_str->B=1;
	else svpwm_str->B=0;
	if(svpwm_str->U3>0) svpwm_str->C=1;
	else svpwm_str->C=0;
	svpwm_str->N = (svpwm_str->C<<2)+(svpwm_str->B<<1)+svpwm_str->A;  //N=C*4+B*2+A
	
	
	//算出XYZ值，方便将调制时间
	svpwm_str->X = Ubeta;
	svpwm_str->Y = 0.5f*(sqrt3*Ualpha+Ubeta);       //标幺后不需要了乘以sqrt3*Ts/Udc！！！！！！
	svpwm_str->Z = 0.5f*(-sqrt3*Ualpha+Ubeta);      //sqrt3*Ts/Udc相同的项没有乘进去，标准的XYZ还需乘以sqrt3*Ts/Udc
	
	//通过N值查表来确定扇区
	switch(svpwm_str->N)
	{
		case 1:
			svpwm_str->T4 = svpwm_str->Z;
			svpwm_str->T6 = svpwm_str->Y;
			svpwm_str->Sector = 2;
			break;
		case 2:
			svpwm_str->T4 = svpwm_str->Y;
			svpwm_str->T6 = -svpwm_str->X;
			svpwm_str->Sector = 6;
			break;
		case 3:
			svpwm_str->T4 = -svpwm_str->Z;
			svpwm_str->T6 = svpwm_str->X;
			svpwm_str->Sector = 1;
			break;
		case 4:
			svpwm_str->T4 = -svpwm_str->X;
			svpwm_str->T6 = svpwm_str->Z;
			svpwm_str->Sector = 4;
			break;
		case 5:
			svpwm_str->T4 = svpwm_str->X;
			svpwm_str->T6 = -svpwm_str->Y;
			svpwm_str->Sector = 3;
			break;
		case 6:
			svpwm_str->T4 = -svpwm_str->Y;
			svpwm_str->T6 = -svpwm_str->Z;
			svpwm_str->Sector = 5;
			break;
		default:             //出现错误，即出现扇区为0的情况
			break;
	}
	
	//过调制判断，当T4+T6>Ts值过调制，就需要缩小比例
	/*
		T4=T4/(T4+T6) * Ts
		T6=T6/(T4+T6) * Ts
	*/
	if((svpwm_str->T4+svpwm_str->T6) > svpwm_str->Ts)
	{
		svpwm_str->T4_temp = svpwm_str->Ts*svpwm_str->T4/(svpwm_str->T4+svpwm_str->T6);
		svpwm_str->T6_temp = svpwm_str->Ts*svpwm_str->T6/(svpwm_str->T4+svpwm_str->T6);
		svpwm_str->T4 = svpwm_str->T4_temp;
		svpwm_str->T6 = svpwm_str->T6_temp;
	}
	
	
	//计算出三相切换点时间
	/*
		Ta=(Ts-T4-T6)/4
		Tb=Ta+T4/2
		Tc=Tb+T6/2
	*/
	svpwm_str->Ta = (svpwm_str->Ts-svpwm_str->T4-svpwm_str->T6)/4.0f;
	svpwm_str->Tb = svpwm_str->Ta+svpwm_str->T4/2.0f;
	svpwm_str->Tc = svpwm_str->Tb+svpwm_str->T6/2.0f;
	
	//根据扇区设置三相比较器的值
	switch(svpwm_str->N)
	{
		case 1:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Tb;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Ta;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Tc;
			break;
		case 2:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Ta;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Tc;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Tb;
			break;
		case 3:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Ta;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Tb;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Tc;
			break;
		case 4:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Tc;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Tb;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Ta;
			break;
		case 5:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Tc;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Ta;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Tb;
			break;
		case 6:
			svpwm_str->PWMA = (uint16_t)svpwm_str->Tb;
			svpwm_str->PWMB = (uint16_t)svpwm_str->Tc;
			svpwm_str->PWMC = (uint16_t)svpwm_str->Ta;
			break;
		default:
			break;
	}
	svpwm_str->PWMA = Limit(svpwm_str->PWMA,1,svpwm_str->Ts-50);
	svpwm_str->PWMB = Limit(svpwm_str->PWMB,1,svpwm_str->Ts-50);
	svpwm_str->PWMC = Limit(svpwm_str->PWMC,1,svpwm_str->Ts-50);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,svpwm_str->PWMA);   //Duty=set/svpwm_str->Ts
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,svpwm_str->PWMB);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,svpwm_str->PWMC);
	
}

//void SVPWM_Adjust(float Uq,float Ud,float elect_angle)
//{
//	float 
//}