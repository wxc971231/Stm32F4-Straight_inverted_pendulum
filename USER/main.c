/*
	直线式倒立摆
		1. 步进电机控制摆位置，用编码器检测步进电机位置
		2. 电位器ADC检测摆杆角度
		3. 按键作为开关
		4. 摆杆角度和水平位置两个闭环控制
		5. 蓝牙遥控控制摆水平位置
*/

#include "sys.h"
#include "delay.h"
#include "uart.h"
#include "led.h"
#include "motor.h"
#include "key.h"
#include "adc.h"
#include "pid.h"
#include "math.h"

#define vertical 675		//垂直时ADC采样

u8 Start_Flag=0;       	//电机锁定/解锁标志
u16 Pluse_High=5;     	//脉冲高电平持续      
u16 Pluse_Period=200;  	//脉冲周期     

extern float adcx;
double Ang_fdb;

//安全控制角度限制----------------------------------------
void safe_ang_limit(void)
{
	Ang_fdb=adcx;
	if(Ang_fdb<(vertical-400) || Ang_fdb>(vertical+400))
		Start_Flag=0;
	
	printf("%f\r\n",Ang_fdb);
}

//检测是否直立，并用led指示,手动立起后解锁-----------------
void Ang_cheak()
{
	if(fabs(Ang_fdb-vertical)<10)
		LED0=0;
	else 
		LED0=1;
}

//按键输入获取--------------------------------------------
u16 Key_Count=0;
void Get_Key()
{
	u8 key;
	
	key=KEY_Scan(0);
	if(key)
	{
		switch(key)
		{				 
			case KEY0_PRES:	//电机解锁
				LED1=0,Start_Flag=1;
				break;
			case KEY1_PRES:	//电机锁定
				LED1=1,Start_Flag=0;
				break;
//			case WKUP_PRES://加速	
//				Pluse_Period-=10;
//				break;
//			case KEY2_PRES://减速
//				Pluse_Period+=10;
//				break;
		}
	}
}

int Encoder[2]={0};
int hold_point;//摆杆水平稳定位置
//u8 hold_point_lock=0;//0->锁定，1->移动
int speed=0;
int mode_flag=0;//0->定点、1->移动

//获取水平速度&位置信息---------------------------------
extern int circle_count;
void Get_Speed(int *Enc,int *Speed)
{
	Enc[0]=Enc[1];
	Enc[1]=TIM_GetCounter(TIM3)+3600*circle_count;
	*Speed = Enc[1]-Enc[0];
}

//从平移进入定点模式--------------------------------------
void Pos_Lock()
{
	static int Pos_Lock_cnt=0;
	Pos_Lock_cnt++;
	if(Pos_Lock_cnt>5)
	{
		mode_flag=0;
		Pos_Lock_cnt=0;
	}
}

float Speed_ref;
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置系统中断优先级分组2	
	My_USART1_Init(115200);													//蓝牙遥控串口
	printf("start");
	delay_init(168);
	LED_Init();			
  Motor_Init();																		//初始化步进电机
	KEY_Init();    				
	Adc_Init();  																		//摆角度电位器
	Encoder_Init();																	//摆位置编码器，这里TIM3作编码器模式
 	TIM4_Int_Init(5-1,84-1);												//84M/84=1Mhz(10^6)计数频率，计数10次为5us  (生成脉冲)
	TIM2_Int_Init(100-1,84-1);											//0.1ms     (ADC)
	TIM5_Init(0xffffffff,84-1);											//1Mhz 做计数器用
	
	delay_ms(100);
	
	hold_point=TIM_GetCounter(TIM3)+3600*circle_count;	//开机位置为默认水平锁定位置	

	Encoder[1]=TIM_GetCounter(TIM3)+3600*circle_count;
	while(1)
	{
		if(Key_Count++>200)
		{
			Key_Count=0;
			Get_Key();
		}
		
		safe_ang_limit();
		Ang_cheak();
		
		Get_Speed(Encoder,&speed);
		
		//杆位置偏差小于100，加pid锁定
		if(fabs(hold_point-Encoder[1])<100)
			Pos_Lock();
		
		//pid计算
		Infantry_Pid_Contrl(vertical,Ang_fdb,hold_point,Encoder[1],speed);
//	Infantry_Pid_Contrl(vertical,Ang_fdb,0,0,0);

		delay_ms(1);
	}
}
