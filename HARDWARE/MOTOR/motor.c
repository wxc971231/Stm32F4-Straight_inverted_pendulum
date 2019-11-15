/*
	来不及了，各种各种模块驱动都写一起了，以后应该拆开到多个文件的 =_=
*/
#include "motor.h"
#include "key.h"
#include "led.h"
#include "pid.h"
#include "adc.h"
#include "uart.h"	

extern u16 Pluse_High;
extern u16 Pluse_Period;
extern u8 Start_Flag;


/*=======================================================外设初始化相关===============================================================*/
//步进电机相关I/O初始化
void Motor_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOE时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6; //DRIVER_DIR DRIVER_OE对应引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOE5,6
	
	dirction=1;//PE5输出高 顺时针方向  DRIVER_DIR
	motor_enable=0;//PE6输出低 使能输出  DRIVER_OE
//---------------------------------------------------------------------------	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //STEP_PULSE 对应引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//速度100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PB8
}

//编码器相关I/O初始化 PA6(CH1),PA7(CH2) 作为编码器A、B项
void Encoder_Init(void)
{
	TIM3_Int_Init();
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

}


//电机加速度控制函数
extern float Ang_OUT;
extern float Pos_OUT;

extern int mode_flag;//0->定点、1->移动
void motor_control(float a,float p)
{
	static float temp_x=1;
	static u8 first_flag=1;
	static float last_p=0;

	if(mode_flag==0)
	{
		first_flag=1;
		temp_x+=a;
		temp_x-=p;
	
	}	
	else
	{
		if(first_flag)
			first_flag=0,temp_x=-p,last_p=-p;
		else
			temp_x+=(p-last_p);
		temp_x+=a;
	}
	
	if(temp_x>0)
		dirction=1,Pluse_Period=10000/temp_x;
	else
		dirction=0,Pluse_Period=-10000/temp_x;
	
	if(Pluse_Period<25)
		Pluse_Period=25;
}



/*=======================================================各种定时器配置===============================================================*/
//TIM4 脉冲生成定时器
void TIM4_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM4时钟
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //设置时钟分割
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM4
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许定时器4更新中断

	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM4,ENABLE); //使能定时器4
}

//TIM3 编码器模式，用于摆位置编码器
void TIM3_Int_Init() 
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟

//定时器设置-------------------------------------------------------------	
  TIM_TimeBaseInitStructure.TIM_Period = 3600; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=0x0;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //设置时钟分割
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
//编码器模式--------------------------------------------------------------	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//???????3
  TIM_ICStructInit(&TIM_ICInitStructure); 
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
//中断设置--------------------------------------------------------------	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断

	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  
	//Reset counter
  TIM_SetCounter(TIM3,0); //TIM3->CNT=0
  TIM_Cmd(TIM3, ENABLE); 
}

//TIM2 生成摆角度电位器的ADC采样周期
void TIM2_Int_Init(u16 arr,u16 psc) 
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //设置时钟分割
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM3
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允许定时器3更新中断

	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x02; //子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2,ENABLE); //使能定时器3
}

//TIM5 生成PID计算周期
void TIM5_Init(uint32_t prd,uint32_t psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  
	
	TIM_TimeBaseInitStructure.TIM_Period = prd; 
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	
	TIM_ARRPreloadConfig(TIM5,ENABLE);
	TIM_Cmd(TIM5,ENABLE); 
}

/*===============================================================各种定时器中断服务函数===============================================================*/
//定时器中断2  ADC扫描
float adcx;					//指示摆杆角度
u16 adc_buff_cnt=0;	//滑动均值滤波
u16 adc_buff[20];		//滑动均值滤波
void TIM2_IRQHandler(void)//0.1ms
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) 	//溢出中断
	{		
		static u16 ADC_Count=0;											//ADC采样周期
		
		ADC_Count++;
		if(ADC_Count>3)															//0.2ms
		{
			ADC_Count=0;
			
			//开机后填充滑动滤波数组
			if(adc_buff_cnt<20)
			{
				adc_buff[adc_buff_cnt]=Get_Adc(ADC_Channel_5);
				adc_buff_cnt++;
			}
			//填充完毕，进行滑动滤波
			else
			{
				for(int i=0;i<19;i++)
					adc_buff[i]=adc_buff[i+1],adcx+=adc_buff[i];
				
				adc_buff[19]=Get_Adc(ADC_Channel_5);
				adcx+=adc_buff[19];
				
				adcx/=20;
			}
		}
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
}



//定时器中断3 编码器计数
int circle_count=0;
void TIM3_IRQHandler(void)//100ms
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{		
		if((TIM3->CR1>>4 & 0x01)==0) //DIR位==0
			circle_count++;
		else if((TIM3->CR1>>4 & 0x01)==1)//DIR位==1
			circle_count--;
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}


//定时器4中断服务函数 脉冲输出
void TIM4_IRQHandler(void)//5us
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //溢出中断
	{	
		static u32 TimeCount=0;
		static u32 AccCount=0;
		
		if(Start_Flag==0)
			motor_enable=1;
		else if(Start_Flag==1)
		{
			motor_enable=0;
			TimeCount++;
			AccCount++;
			
			if(TimeCount<Pluse_High)//脉冲高电平持续10us
				Pluse=1;
			else if(TimeCount>Pluse_High)
				Pluse=0;
			if(TimeCount>Pluse_Period)//周期
				TimeCount=0;
			
		}
		
		if(AccCount>5)	//0.1ms
		{
			AccCount=0;
			motor_control(Ang_OUT/1000,Pos_OUT/1000);
		}	
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //清除中断标志位
}




