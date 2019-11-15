#include "pid.h"
#include "sys.h"
#include "delay.h"
/*
#define Pkp 0.4   //控制水平位置定点 
#define Pki 0
#define Pkd 1

#define Pkp_s 1 //遥控时平移过程控制
#define Pki_s 0
#define Pkd_s 0

#define Akp 50   //控制稳摆角度
#define Aki 0
#define Akd 30 
*/
//800细分---------------------
#define Pkp 0.095   //控制水平位置定点 
#define Pki 0
#define Pkd 0.15  

#define Pkp_s 1.5 //遥控时平移过程控制
#define Pki_s 0
#define Pkd_s 1.2

#define Akp 15   //控制稳摆角度
#define Aki 0
#define Akd 8 
//----------------------------
PIDStrucTypeDef Ang_Pid={0,0,Akp,Aki,Akd,0.001,0,0,0,100,-100,0,0};
PIDStrucTypeDef Pos_Pid={0,0,Pkp,Pki,Pkd,0.001,0,0,0,100,-100,0,0};
PIDStrucTypeDef Speed_Pid={0,0,Pkp_s,Pki_s,Pkd_s,0.001,0,0,0,100,-100,0,0};

float PID_Cal(PIDStrucTypeDef *PID)
{
	PID->Ek=PID->ref-PID->fdb;
	PID->Sk+=PID->Ek*PID->dt;
	PID->out=1.0*(PID->kp*PID->Ek+PID->ki*PID->Sk+PID->kd*PID->Dk);
	PID->Ek_1=PID->Ek_2;
	PID->Ek_2=PID->Ek;
	
	PID->Dk=(PID->Ek_2-PID->Ek_1)/PID->dt;
	return PID->out/10;
}

void Set_ref_fdb(PIDStrucTypeDef *PID,double REF,double FDB)
{
	PID->ref=REF;
	PID->fdb=FDB;
}

void Set_dt(PIDStrucTypeDef *PID,double DT)
{
	PID->dt=DT;
}
/*
float Ang_PID_cal(float REF,float FDB)
{
	Ang_Pid.Ek=REF-FDB;
	Ang_Pid.Sk+=Ang_Pid.Ek*Ang_Pid.dt;
	Ang_Pid.out=1.0*(Akp*Ang_Pid.Ek+Aki*Ang_Pid.Sk+Akd*Ang_Pid.Dk);
//	if(Pout>PoutMax)
//		Pout=PoutMax;
//	if(Pout<PoutMin)
//		Pout=PoutMin;
	
	Ang_Pid.Ek_1=Ang_Pid.Ek_2;
	Ang_Pid.Ek_2=Ang_Pid.Ek;
	
	Ang_Pid.Dk=(Ang_Pid.Ek_2-Ang_Pid.Ek_1)/Ang_Pid.dt; 
	
	return Ang_Pid.out/10;
}

float Pos_PID_cal(float REF,float FDB)
{
	Pos_Pid.Ek=REF-FDB;
	Pos_Pid.Sk+=Pos_Pid.Ek*Pos_Pid.dt;
	Pos_Pid.out=1.0*(Pkp*Pos_Pid.Ek+Pki*Pos_Pid.Sk+Pkd*Pos_Pid.Dk);
//	if(Sout>SoutMax)
//		Sout=SoutMax;
//	if(Sout<SoutMin)
//		Sout=SoutMin;
	
	Pos_Pid.Ek_1=Pos_Pid.Ek_2;
	Pos_Pid.Ek_2=Pos_Pid.Ek;
	
	Pos_Pid.Dk=(Pos_Pid.Ek_2-Pos_Pid.Ek_1)/Pos_Pid.dt; 
	
	return Pos_Pid.out/10;
}

float Speed_PID_cal(float REF,float FDB)
{
	Speed_Pid.Ek=REF-FDB;
	Speed_Pid.Sk+=Speed_Pid.Ek*Speed_Pid.dt;
	Speed_Pid.out=1.0*(Pkp_s*Speed_Pid.Ek+Pkp_s*Speed_Pid.Sk+Pkp_s*Speed_Pid.Dk);
//	if(Sout>SoutMax)
//		Sout=SoutMax;
//	if(Sout<SoutMin)
//		Sout=SoutMin;
	
	Speed_Pid.Ek_1=Speed_Pid.Ek_2;
	Speed_Pid.Ek_2=Speed_Pid.Ek;
	
	Speed_Pid.Dk=(Speed_Pid.Ek_2-Speed_Pid.Ek_1)/Speed_Pid.dt; 
	
	return Speed_Pid.out/30;
}
*/

//下面是获取采样周期----------------------------------------------------------------
//get present microsecond.
uint32_t micros(void)
{
	uint32_t time_now=TIM_GetCounter(TIM5);
	return time_now;
}

uint32_t spd_time_now=0,spd_time_last=0;
float pid_dt;
void get_dt_in_seconds(u32 *time_now,u32 *time_last,float *dt)
{
	*time_now=micros();
	if(*time_now <= *time_last) {   //计数值溢出了
        *dt = (float)(*time_now + (0xffffffff - *time_last))/220000.0f;
    }
	else{
        *dt = (float)(*time_now - *time_last)/220000.0f;
	}
	*time_last=*time_now;
}

extern int mode_flag;//0->定点、1->移动
float Ang_OUT;
float Pos_OUT;
float Position_out;
void Infantry_Pid_Contrl(float Ang_Ref,float Ang_Fdb,float Pos_Ref,float Pos_Fdb,float Speed_Fdb)
{
	//dt第一次设0.001，之后用定时器测算----------------------------------------------
	static u8 fun_first_run=1;
	get_dt_in_seconds(&spd_time_now,&spd_time_last,&pid_dt);
	if(fun_first_run)
	{
		fun_first_run=0;
		pid_dt=0.001;
	}
	Set_dt(&Ang_Pid,pid_dt);
	Set_dt(&Pos_Pid,pid_dt);
	Set_dt(&Speed_Pid,pid_dt);
	
	Set_ref_fdb(&Ang_Pid,Ang_Ref,Ang_Fdb);
	Set_ref_fdb(&Pos_Pid,Pos_Ref,Pos_Fdb);
	
	if(mode_flag==0)
	{
		Ang_OUT=PID_Cal(&Ang_Pid);
		Pos_OUT=PID_Cal(&Pos_Pid);
	}
	else if(mode_flag==1)
	{
		Ang_OUT=PID_Cal(&Ang_Pid);
		Position_out=PID_Cal(&Pos_Pid);
		
		Set_ref_fdb(&Speed_Pid,Position_out,Speed_Fdb);
		Pos_OUT=-PID_Cal(&Speed_Pid)/10;
	}
//	Ang_OUT=0;
}

