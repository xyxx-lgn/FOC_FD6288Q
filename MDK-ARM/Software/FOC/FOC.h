#ifndef __FOC_H
#define __FOC_H		


//头文件包含
#include "ALL_H.h"



float ElectAngle_Turn(float RawAngle,uint8_t pole);
float ElectAngle_Limit(float Angle);
void SetPwm(float Ua,float Ub,float Uc);
void SetPhaseVoltage(float Uq,float Ud,float elect_angle);
void Clark_Park(AdcValue *adcvalue,Encoder_Struct *encoder_str,PID *pid);
void SVPWM(float Uq,float Ud,float Angle,SVPWM_Struct *svpwm_str);
#endif
