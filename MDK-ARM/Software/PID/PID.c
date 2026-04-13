#include "PID.h"
#include "Motor.h"

extern PID pid_m1;                     //PID参数结构体
extern Encoder_Struct encoder_str;     //编码器结构体
extern Filter Speed_LowPass;           //速度环速度低通滤波器
extern Filter Speed_LowPass1;           //速度环速度低通滤波器

void PID_I_Control(PID *pid)           //电流环控制(PI控制器)
{
	/*
		BEMF 电压 E = ω·Ke
		电机额定 3000 rpm
		KV = 170 rpm/V（机械）
		pole = 7
		Ke_rms = 1 / (KV × √3 × √2) × 60/(2π)
			   = 1 / (170 × 1.732 × 1.414) × 9.549
               ≈ 0.00229 V·s/rad（相电压有效值）

		Ke_peak = Ke_rms × √2
                = 0.00229 × 1.414
                ≈ 0.00324 V·s/rad
	*/
//	float omega_e = pid->Speed_now*0.733038f;   //机械->电  pid->Speed_now*7*2.0f*PI/60.0f，变成rad/s  0.733038
//	float BEMF = omega_e*0.00324f;                      //峰值
	
	//电流环PI控制
	//Iq
	pid->erro_iq = pid->Iq_aim-pid->Iq_current;                                //电流iq误差
	pid->erro_iq_sum = pid->erro_iq_sum+pid->Ki_iq*pid->erro_iq;               //积分累加
	pid->erro_iq_sum = Limit(pid->erro_iq_sum,-pid->Ki_SumMax,pid->Ki_SumMax); //积分限幅
	pid->Uq = pid->Kp_iq*pid->erro_iq+pid->erro_iq_sum;                        //电压Uq计算
	pid->Uq = Limit(pid->Uq,-pid->Ki_SumMax,pid->Ki_SumMax);                   //电压Uq限幅
//	pid->Uq += BEMF;                                                           //补偿
	//Id
	pid->erro_id = pid->Id_aim-pid->Id_current;                                //电流id误差
	pid->erro_id_sum = pid->erro_id_sum+pid->Ki_iq*pid->erro_id;               //积分累加
	pid->erro_id_sum = Limit(pid->erro_id_sum,-pid->Ki_SumMax,pid->Ki_SumMax); //积分限幅
	pid->Ud = pid->Kp_iq*pid->erro_id+pid->erro_id_sum;                        //电压Ud计算
	pid->Ud = Limit(pid->Ud,-pid->Ki_SumMax,pid->Ki_SumMax);                   //电压Ud限幅	
	
	
}

void PID_Speed_Control(PID *pid,Encoder_Struct *encoder_str)
{

	

	
	if(pid->speed_count++>9)
	{
		encoder_str->Encoder_raw_erro = (encoder_str->Encoder_raw-encoder_str->Encoder_old_raw);
		//16384/2，判断正反转过零点
		if(encoder_str->Encoder_raw_erro>8192)          //反转过零点
			encoder_str->Encoder_raw_erro = encoder_str->Encoder_raw-encoder_str->Encoder_old_raw-16384;
		else if(encoder_str->Encoder_raw_erro<-8192)    //正转过零点  
			encoder_str->Encoder_raw_erro = 16384-encoder_str->Encoder_old_raw+encoder_str->Encoder_raw;
		
		encoder_str->Encoder_raw_sum += (float)encoder_str->Encoder_raw_erro;
		
		encoder_str->Encoder_old_raw = encoder_str->Encoder_raw;               //保留上一次编码器数值
		
		pid->Speed_now = encoder_str->Encoder_raw_sum*7.3242187f;  //(Encoder_raw_sum/16384)*(2000*60) 为转/分
		pid->Speed_now = Low_pass_Filter(&Speed_LowPass,pid->Speed_now);
		pid->Speed_show = pid->Speed_now;
		pid->Speed_show = Low_pass_Filter(&Speed_LowPass1,pid->Speed_show);
		encoder_str->Encoder_raw_sum = 0;
		pid->speed_count = 0;
		
		pid->Speed_aim = Limit(pid->Speed_aim,-pid->speed_max,pid->speed_max);   //输入速度限幅
		pid->erro_speed = pid->Speed_aim-pid->Speed_now;
		pid->erro_speed_sum = pid->erro_speed_sum+pid->Ki_speed*pid->erro_speed;
		pid->erro_speed_sum = Limit(pid->erro_speed_sum,-pid->speed_out_max,pid->speed_out_max); //积分限幅
		pid->speed_out = pid->Kp_speed*pid->erro_speed+pid->erro_speed_sum;
		pid->speed_out =Limit(pid->speed_out,-pid->speed_out_max,pid->speed_out_max);            //输出Iq限幅
	}
	

//		//速度PI环
//		pid->Speed_aim = Limit(pid->Speed_aim,-pid->speed_max,pid->speed_max);   //输入速度限幅
//		pid->erro_speed = pid->Speed_aim-pid->Speed_now;
//		pid->erro_speed_sum = pid->erro_speed_sum+pid->Ki_speed*pid->erro_speed;
//		pid->erro_speed_sum = Limit(pid->erro_speed_sum,-pid->speed_out_max,pid->speed_out_max); //积分限幅
//		pid->speed_out = pid->Kp_speed*pid->erro_speed+pid->erro_speed_sum;
//		pid->speed_out =Limit(pid->speed_out,-pid->speed_out_max,pid->speed_out_max);            //输出Iq限幅

//	pid->Speed_show = alpha*pid->Speed_now+(1-alpha)*pid->Speed_last;
//	pid->Speed_last = pid->Speed_show;
//	encoder_str->Encoder_raw_sum = 0;

	//速度环执行周期为电流环的5~10倍,我这里10倍，即频率为1K
//	if(pid->speed_count++>9)
//	{
//		pid->Speed_now = (float)encoder_str->Encoder_raw_sum*3.662109f;  //(Encoder_raw_sum/16384)*(1000*60) 为转/分
//		pid->Speed_show = alpha*pid->Speed_now+(1-alpha)*pid->Speed_last;
//		pid->Speed_last = pid->Speed_show;
//		encoder_str->Encoder_raw_sum = 0;
//		pid->speed_count = 0;
//		//速度PI环
//		pid->Speed_aim = Limit(pid->Speed_aim,-pid->speed_max,pid->speed_max);   //输入速度限幅
//		pid->erro_speed = pid->Speed_aim-pid->Speed_now;
//		pid->erro_speed_sum = pid->erro_speed_sum+pid->Ki_speed*pid->erro_speed;
//		pid->erro_speed_sum = Limit(pid->erro_speed_sum,-pid->speed_out_max,pid->speed_out_max); //积分限幅
//		pid->speed_out = pid->Kp_speed*pid->erro_speed+pid->erro_speed_sum;
//		pid->speed_out =Limit(pid->speed_out,-pid->speed_out_max,pid->speed_out_max);            //输出Iq限幅
//	}
}

void PID_Position_Control(PID *pid,Encoder_Struct *encoder_str)
{
	pid->Position_aim = fmod(pid->Position_aim,360);
	if(pid->Position_aim<0) pid->Position_aim+=360;
	pid->erro_positon = pid->Position_aim-encoder_str->Shaft_Angle;                                //角度误差
	pid->erro_position_sum = pid->erro_position_sum+pid->Ki_position*pid->erro_positon;               //积分累加
	pid->erro_position_sum = Limit(pid->erro_position_sum,-pid->position_out_max,pid->position_out_max); //积分限幅
	pid->position_out = pid->Kp_position*pid->erro_positon+pid->erro_position_sum;                        //输出速度计算
	pid->position_out = Limit(pid->position_out,-pid->position_out_max,pid->position_out_max);                   //输出速度限幅	
}

