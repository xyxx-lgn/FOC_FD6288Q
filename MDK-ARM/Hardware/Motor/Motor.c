#include "Motor.h"
#include "MT6701.h"
#include "FOC.h"
#include "PID.h"
#include "Test.h"


extern AdcValue adcvalue;              //ADC采样变量
extern Encoder_Struct encoder_str;     //编码器结构体
extern SVPWM_Struct svpwm_str;         //SVPWM结构体
extern PID pid_m1;                     //PID参数结构体
extern Motor_Test Motor;               //电机测试变量
extern Filter Adc_FilterA;              //ADC的一阶低通滤波器A相
extern Filter Adc_FilterB;              //ADC的一阶低通滤波器B相
extern Filter Adc_FilterC;              //ADC的一阶低通滤波器C相
extern Filter Speed_LowPass;           //速度环速度低通滤波器
extern Filter Speed_LowPass1;           //速度环速度低通滤波器


void Data_Init()
{
	//adcvalue结构体初始化
	adcvalue.supply_Udc=22.2f;           //输入电压值最大值
	adcvalue.Ia_Sample=0.0f;
	adcvalue.Ib_Sample=0.0f;
	adcvalue.Ic_Sample=0.0f;
	adcvalue.Udc_Sample=0.0f;           //母线电压采样值
	adcvalue.Ia_offect=1.65f;
	adcvalue.Ib_offect=1.65f;
	adcvalue.Ic_offect=1.65f;
	adcvalue.Ia=0.0f;
	adcvalue.Ib=0.0f;
	adcvalue.Ic=0.0f;
	adcvalue.Udc=0.0f;
	adcvalue.Iadc_count=0;
	adcvalue.Gain_I=0.25f;        //电流增益倍数 Gain*Rs=50*0.005=0.25
	
	//allflag结构体初始化
	allflag.Drv8301_flag=1;      //Drv8301初始化标志位，0代表未初始化，1代表初始化成功，2代表初始化失败   
	allflag.Adc_Adjust_flag=0;   //Adc校准标志位，0代表未校准，1代表校准完成，2代表过流
	allflag.Zero_flag=1;         //编码器零点校准标志位,0代表未校准，1代表校准完成  
	allflag.Encoder_flag=0;      //编码器模式 1.编码器开环控制 2.编码器闭环控制
	allflag.Mode_flag=0;         //电机控制模式选择
	
	//svpwm_str结构体初始化
	svpwm_str.Udc=22.2f;
	svpwm_str.U1=0.0f;
	svpwm_str.U2=0.0f;
	svpwm_str.U3=0.0f;
	svpwm_str.A=0;
	svpwm_str.B=0;
	svpwm_str.C=0;
	svpwm_str.N=0;
	svpwm_str.Sector=0;
	svpwm_str.X=0.0f;
	svpwm_str.Y=0.0f;
	svpwm_str.Z=0.0f;
	svpwm_str.Ts=4250;      //PWM寄存器值
	svpwm_str.T4=0.0f;
	svpwm_str.T6=0.0f;
	svpwm_str.T4_temp=0.0f;
	svpwm_str.T6_temp=0.0f;
	svpwm_str.Ta=0.0f;
	svpwm_str.Tb=0.0f;
	svpwm_str.Tc=0.0f;
	svpwm_str.PWMA=0;
	svpwm_str.PWMB=0;
	svpwm_str.PWMC=0;
	
	//编码器结构体
	encoder_str.motordir=0;              //电机方向 取值{0,1}
	encoder_str.r_s_speed=0;              //开环设置的转速，单位：圈/秒(指的是机械角度)
	encoder_str.Encoder_raw=0;  		  //编码器原始值
	encoder_str.Encoder_old_raw=0;        //上一次编码器原始值
	encoder_str.Encoder_raw_erro=0;       //编码器误差值
	encoder_str.Encoder_raw_sum=0;        //编码器原始值和
	encoder_str.Shaft_Angle=0.0f;         //机械角度
	encoder_str.Elect_Angle=0.0f;         //电角度
	encoder_str.Encoder_Mode1_Angle=0.0f; //模式1临时角度变量
	encoder_str.Encoder_Mode3_Angle=10.0f;//模式3临时角度变量
	encoder_str.Return_Angle=0.0f;        //真实返回角度(输出给SVPWM)
	encoder_str.Zero_Angle_cal=0.0f;      //零点偏移角度计算中间变量
	encoder_str.Zero_Angle=0.0f;          //零点偏移角度
	encoder_str.zero_count=0;             //编码器方向校准和零点校准
	
	//PID结构体
	//下面为电流环参数
	pid_m1.Kp_iq=0.0924f;            //iq的Kp值        相电感*350*2*pi=0.000042*350*2*pi=0.0924f   里面的350是指电角度多少转每秒，换算为350/7*60=3000转每分(机械角度)
	pid_m1.Ki_iq=0.0087964f;         //iq的Ki值        相电阻*350*2*pi/电流环执行频率=0.08*350*2*pi/20000=0.0087964f

	pid_m1.Iq_aim=0.0f;        //iq目标值
	pid_m1.Iq_current=0.0f;    //iq实际值
	pid_m1.erro_iq=0.0f;       //PI控制器里面的误差，等于目标值-实际值
	pid_m1.erro_iq_sum=0.0f;   //Ki的积分项
	pid_m1.Uq=0.0f;            //输出的Uq值
	
	pid_m1.Ki_SumMax=0.0f;     //KI的积分限幅最大值  Udc/sqrt(3),一般再乘以0.9,但本工程出问题，具体看pid_m1.speed_out_max处注释
	pid_m1.Ialfa=0.0f;         //电流环帕克变换
	pid_m1.Ibeta=0.0f;
	pid_m1.Vmax=0.0f;          //电流环输出最大电压矢量

	pid_m1.Id_aim=0.0f;        //id目标值
	pid_m1.Id_current=0.0f;    //id实际值
	pid_m1.erro_id=0.0f;       //PI控制器里面的误差，等于目标值-实际值
	pid_m1.erro_id_sum=0.0f;   //Ki的积分项
	pid_m1.Ud=0.0f;            //输出的Ud值
	
	//速度环参数
	pid_m1.Kp_speed=0.005f;      //速度环Kp值       0.005f
	pid_m1.Ki_speed=0.000001f;      //速度环Ki值       0.000001f
	
	pid_m1.Speed_aim=0.0f;     //目标速度，单位：转/分 
	pid_m1.Speed_last=0.0f;    //实际速度
	pid_m1.Speed_now=0.0f;     //实际速度
	pid_m1.Speed_show=0.0f;    //滤波后速度
	pid_m1.erro_speed=0.0f;    //速度误差
	pid_m1.erro_speed_sum=0.0f;//速度积分误差
	pid_m1.speed_out=0.0f;     //PI输出结果，输出为电流环Iq_aim
	
	pid_m1.speed_out_max=3.0f; //速度环输出Iq限幅值   Imax=(3.3/2)/(G*S)=1.65/0.8=2.06, 一般取0.9，但是测试发现GVDD欠压有问题，可能电容原因，只能取1A，因此要乘系数为1/2.06
	pid_m1.speed_max=4000;      //速度设定限幅  [-360,360]
	pid_m1.speed_count=0;      //速度环计次
	
	//下面为位置环
	pid_m1.Kp_position=9.2f;        //位置环Kp值
	pid_m1.Ki_position=0.00005f;        //位置环Ki值 
	
	pid_m1.Position_aim=180.0f;       //目标位置，取值[0,360]
	pid_m1.erro_positon=0.0f;       //位置误差=Position_aim-Shaft_Angle
	pid_m1.erro_position_sum=0.0f;  //位置积分误差 
	pid_m1.position_out=0.0f;       //位置环PI输出，输出为速度环的Speed_aim  
	
	pid_m1.position_out_max=2000.0f;   //位置环的输出限幅
	
	
	//Motor_Test结构体
	Motor.Iwidth=300.0f;           //电流环带宽
	Motor.Set=350.0f;
	
	//ADC的一阶低通滤波器 
	Adc_FilterA.alpha_filter=0.2390572f;    //时间常数   A=1/[1+1/(2*PI*fc*T)]=0.2390572f   0.1357552f
	Adc_FilterA.last_output=0.0f;           //上一时刻输出
	
	Adc_FilterB.alpha_filter=0.2390572f;    //时间常数     0.0861302f
	Adc_FilterB.last_output=0.0f;           //上一时刻输出
	
	Adc_FilterC.alpha_filter=0.2390572f;    //时间常数     0.0861302f
	Adc_FilterC.last_output=0.0f;           //上一时刻输出
	
	//速度环的一阶低通滤波器
	Speed_LowPass.alpha_filter=0.2390572f;       //采样周期T=1/2k=0.0005s  A=0.7585469
	Speed_LowPass.last_output=0.0f;
	
	Speed_LowPass1.alpha_filter=0.2390572f;       //采样周期T=1/2k=0.0005s  A=0.7585469
	Speed_LowPass1.last_output=0.0f;
}

	float theta;

/********************
一阶低通滤波器：
	Y(k)=A*X(k)+(1-A)*Y(k-1)
	A=1/[1+1/(2*PI*fc*T)]     1/[1+1/(2*3.1415926f*1000*0.00005)]=0.2390572f
	
	fc=300:    0.0861302f
	fc=500:    0.1357552f
	fc=1k:     0.2390572f

	Y(k):输出
	Y(k-1):上一次输出
	X(k):输入
	fc:截止频率
	T:采样周期
********************/
float Low_pass_Filter(Filter* Adc_Filter,float input)
{
	//计算输出
	float Low_pass_output = Adc_Filter->alpha_filter*input+(1.0f-Adc_Filter->alpha_filter)*Adc_Filter->last_output;
	//更新上一次输出
	Adc_Filter->last_output = Low_pass_output;
	return Low_pass_output;
}


/********************
限幅函数，用于浮点数限制，一般用于角度弧度限制
	超限一般加减一个周期
********************/
float Angle_Limit(float raw,float Limit)
{
	if(raw>Limit)
		raw -= Limit;
	else if(raw<0)
		raw += Limit;
	return raw;
}

/********************
ADC数据采样处理
	电流计算公式
		Vo=1/2Vref-G*V(Sn-Sp)
		Vrs=V(Sn-Sp)
		即-(Vo-1/2Vref)/G=Vrs
		因此I=Vrs/Rs(采样电阻值)
		I=-(Vo-1/2Vref)/(G*Rs)   ,此处的Vref最好是先校准，一般不是1.65V
********************/
void Adcpro(AdcValue *adcvalue,AllFlag *allflag,uint16_t *adc_raw)
{
	adcvalue->Ia_Sample = adc_raw[0];  //A相adc采样值
	adcvalue->Ib_Sample = adc_raw[1];  //B相adc采样值
	adcvalue->Ic_Sample = adc_raw[2];  //C相adc采样值
	
	adcvalue->Ia_Sample = Low_pass_Filter(&Adc_FilterA,adcvalue->Ia_Sample);
	adcvalue->Ib_Sample = Low_pass_Filter(&Adc_FilterB,adcvalue->Ib_Sample);
	adcvalue->Ic_Sample = Low_pass_Filter(&Adc_FilterC,adcvalue->Ic_Sample);
//	
	adcvalue->Udc_Sample = adc_raw[3]; //Udc电压adc采样值
	
	//1.先对电流ADC偏置值校准
	if(allflag->Drv8301_flag==1 && allflag->Adc_Adjust_flag==0)    //电流adc未校准
	{
		if(adcvalue->Iadc_count<10000)  //PWM频率20K，进行10000次，用时0.5s
		{
			adcvalue->Ia = adcvalue->Ia_Sample*3.3f/4096.0f;
			adcvalue->Ib = adcvalue->Ib_Sample*3.3f/4096.0f;
			adcvalue->Ic = adcvalue->Ic_Sample*3.3f/4096.0f;
			
			adcvalue->Ia_offect = adcvalue->Ia_offect*0.99f+adcvalue->Ia*0.01f;      //一阶滤波
			adcvalue->Ib_offect = adcvalue->Ib_offect*0.99f+adcvalue->Ib*0.01f;
			adcvalue->Ic_offect = adcvalue->Ic_offect*0.99f+adcvalue->Ic*0.01f;
			
			adcvalue->Iadc_count++;
		}
		else
		{
			allflag->Adc_Adjust_flag = 1;     //校准完成
			adcvalue->Iadc_count = 0;
		}
	}
	else if(allflag->Drv8301_flag==1 && allflag->Adc_Adjust_flag==1)
	//ADC系数转换 
	{   //由于FD6288电流方向不一样，所以取消电流值前的符号，否则电流环的PI参数改为负值
		adcvalue->Ia = ((adcvalue->Ia_Sample*3.3f/4096.0f-adcvalue->Ia_offect)/adcvalue->Gain_I);    
		adcvalue->Ib = ((adcvalue->Ib_Sample*3.3f/4096.0f-adcvalue->Ib_offect)/adcvalue->Gain_I);
		adcvalue->Ic = ((adcvalue->Ic_Sample*3.3f/4096.0f-adcvalue->Ic_offect)/adcvalue->Gain_I);   //三电阻采样
//		adcvalue->Ic = -(adcvalue->Ia+adcvalue->Ib);                                                 //双电阻采样
		
		
		//(Udc_Sample-504)/4096*3.3 * (4.99+34.8)/4.99 =Udc,这个504是我自己给的一个补偿值，这里有点问题，理论上应该是减0，不减的话电压太大已经大于供电电压了
		//0.00642432326=1/4096*3.3 * (4.99+34.8)/4.99
		//母线电压采集这里的电阻值有点问题，所以得自减一个偏差值
		adcvalue->Udc = (adcvalue->Udc_Sample-0)*0.0088623f;   //母线电压采样的一阶低通滤波的截止频率一般设置为采样周期的[1/20，1/10]  0.00642432326f
        if(adcvalue->Udc>adcvalue->supply_Udc)   //防止意外超出
			adcvalue->Udc=adcvalue->supply_Udc;
		svpwm_str.Udc = adcvalue->Udc;
		pid_m1.Ki_SumMax = adcvalue->Udc*one_sqrt3*0.8f;    
//		pid_m1.Vmax = adcvalue->Udc*one_sqrt3;
	}
}


/********************
磁编码器数据处理
********************/
void Encoderpro(Encoder_Struct *encoder_str,AllFlag *allflag)
{
//	encoder_str->Encoder_raw = (int16_t)MT6701_ReadRaw();                           //编码器原始数据获取 0-16384
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
	encoder_str->Encoder = MT6701_ReadRaw();
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
	if(encoder_str->motordir ==1)
		encoder_str->Encoder_raw = 16384-encoder_str->Encoder;
	else
		encoder_str->Encoder_raw = encoder_str->Encoder;
	encoder_str->Shaft_Angle = (float)encoder_str->Encoder_raw * 0.0219726f -21.0717f ;   //机械角度获取 21.0717 
	encoder_str->Shaft_Angle = Angle_Limit(encoder_str->Shaft_Angle,360.f);
	
	encoder_str->Elect_Angle = ElectAngle_Limit(encoder_str->Shaft_Angle); //电角度获取，0-360
	encoder_str->Elect_Angle = Angle_Limit(encoder_str->Elect_Angle,360.f);
	
//	encoder_str->Encoder_raw_erro = (encoder_str->Encoder_raw-encoder_str->Encoder_old_raw);
//	//16384/2，判断正反转过零点
//	if(encoder_str->Encoder_raw_erro>8192)          //反转过零点
//		encoder_str->Encoder_raw_erro = encoder_str->Encoder_raw-encoder_str->Encoder_old_raw-16384;
//	else if(encoder_str->Encoder_raw_erro<-8192)    //正转过零点  
//		encoder_str->Encoder_raw_erro = 16384-encoder_str->Encoder_old_raw+encoder_str->Encoder_raw;
//	
//	encoder_str->Encoder_raw_sum += (float)encoder_str->Encoder_raw_erro;
//	
//	encoder_str->Encoder_old_raw = encoder_str->Encoder_raw;               //保留上一次编码器数值
	
	
	if(allflag->Adc_Adjust_flag==1 && allflag->Zero_flag==0)               //ADC电流采样校准完成后进行磁编码器校准
	{
		SVPWM(0,1,0,&svpwm_str);
		encoder_str->zero_count++;
		if(encoder_str->zero_count>19500)        //让对齐0.95s后再校准
		{
			encoder_str->Zero_Angle_cal += encoder_str->Shaft_Angle;
			if(encoder_str->zero_count>=20000)  //0.1s采集求平均值 
			{
				encoder_str->Zero_Angle = encoder_str->Zero_Angle_cal/500.0f;
				encoder_str->Zero_Angle_cal = 0;
				encoder_str->zero_count = 0;
				allflag->Zero_flag = 1;
				
			}
		}	
	}
	
	if(allflag->Encoder_flag==1)  //编码器模式1：开环角度自增(方向由motordir决定)
	{
		encoder_str->r_s_speed = 2;         //转/s
		encoder_str->Encoder_Mode1_Angle += encoder_str->r_s_speed*0.018f;  
		encoder_str->Encoder_Mode1_Angle = Angle_Limit(encoder_str->Encoder_Mode1_Angle,360.f);//机械角度
  
		encoder_str->Return_Angle = ElectAngle_Limit(encoder_str->Encoder_Mode1_Angle);        //电角度
		
		encoder_str->Return_Angle = encoder_str->Return_Angle*2*PI/360.0f;
		encoder_str->Return_Angle = Angle_Limit(encoder_str->Return_Angle,2*PI);
	}
	else if(allflag->Encoder_flag==2) //编码器模式2：闭环角度控制  返回校准后的电角度值
	{
		encoder_str->Return_Angle = encoder_str->Elect_Angle;
		encoder_str->Return_Angle = encoder_str->Return_Angle*2*PI/360.0f;
		encoder_str->Return_Angle = Angle_Limit(encoder_str->Return_Angle,2*PI);
	}
	else if(allflag->Encoder_flag==3) //编码器模式3：角度定点控制 取值[0,360]
	{
		encoder_str->Encoder_Mode3_Angle = 40; //设定角度
		
		encoder_str->Return_Angle = ElectAngle_Limit(encoder_str->Encoder_Mode3_Angle);        //电角度
		encoder_str->Return_Angle = encoder_str->Return_Angle*2*PI/360.0f;
		encoder_str->Return_Angle = Angle_Limit(encoder_str->Return_Angle,2*PI);
	}
	
	

}

/********************
电机控制模式选择
********************/
void Modepro(Encoder_Struct *encoder_str,AllFlag *allflag)     
{
	//电压开环控制
	if(allflag->Mode_flag==1)
	{
		allflag->Encoder_flag = 1;
		SVPWM(0.5,0,encoder_str->Return_Angle,&svpwm_str);
	}
	//电流环控制
	else if(allflag->Mode_flag==2)
	{
//	pid_m1.Kp_iq=0.000263894f*Motor.Set;	
//	pid_m1.Ki_iq=0.0000251327f*Motor.Set;

	theta += Motor.Iwidth*2*PI/20000;
	theta = Angle_Limit(theta,2*PI);
	
	allflag->Encoder_flag = 3;
//	pid_m1.Iq_aim = 0.0f;
	pid_m1.Id_aim = encoder_str->Encoder_Mode1_Angle*arm_sin_f32(theta);
	PID_I_Control(&pid_m1);
	SVPWM(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
		
//		Cal_IWidth();          //电流环带宽检测，使用该函数时，模式里面其他行注释掉，二者留其一
		
//		pid_m1.Iq_aim = 0.2f;
//		pid_m1.Id_aim = 0.0f;
		
//		allflag->Encoder_flag = 2;
//		PID_I_Control(&pid_m1);
//		SVPWM(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
	}
	//速度环-电流环
	else if(allflag->Mode_flag==3)
	{		
		allflag->Encoder_flag = 2;
//		pid_m1.Speed_aim = 120;
		PID_Speed_Control(&pid_m1,encoder_str);      //速度环
		pid_m1.Iq_aim = pid_m1.speed_out;
		pid_m1.Id_aim = 0;
		PID_I_Control(&pid_m1);
		SVPWM(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
	}
	//位置环-速度环-电流环
	else if(allflag->Mode_flag==4)
	{
		allflag->Encoder_flag = 2;
//		pid_m1.Position_aim=120.0f;
		PID_Position_Control(&pid_m1,encoder_str);    //位置环
		pid_m1.Speed_aim = pid_m1.position_out;
		PID_Speed_Control(&pid_m1,encoder_str);      //速度环
		pid_m1.Iq_aim = pid_m1.speed_out;
		pid_m1.Id_aim = 0;
		PID_I_Control(&pid_m1);
		SVPWM(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
	}
}

/********************
ADC数据采样处理和编码器采样数据处理
********************/
void Data_Treating()
{
	
	Adcpro(&adcvalue,&allflag,ADC1InjectDate);
	Encoderpro(&encoder_str,&allflag);
	Clark_Park(&adcvalue,&encoder_str,&pid_m1);   //用于计算电流环的Iq和Id
 
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
//	SVPWM(pid_m1.Uq,pid_m1.Ud,0,&svpwm_str);
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
	
	allflag.Mode_flag=3;        //控制模式选择
  if(allflag.Adc_Adjust_flag==1 && allflag.Zero_flag==1)
  {
	Modepro(&encoder_str,&allflag);
  }
}
