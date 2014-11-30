/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：moto.c
 * 描述    ：电机驱动配置         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "moto.h"


void Tim2_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/**********************************************************
	72 000 000/72=1M
	1000 000/2500=400Hz
	所以产生的PWM为400Hz
	周期为2.5ms，对应2500的计算值，1ms~2ms对应的计算值为1000~2000；
	**********************************************************/
	TIM_TimeBaseStructure.TIM_Period = 3599;		//计数上线	
	TIM_TimeBaseStructure.TIM_Prescaler = 8;	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;//初始占空比为0
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void Moto_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA and GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	Tim2_init();	
}


void moto_PwmRflash(int16_t *Moter)
{		
	for(u8 i=0;i<4;i++)
	{
     if(*(Moter+i) > Moto_PwmMax)  *(Moter+i) = Moto_PwmMax;
  }
	for(u8 i=0;i<4;i++)
	{
     if(*(Moter+i) <= 0 )  *(Moter+i) = 0;
  }
	TIM2->CCR1 = *(Moter++);
	TIM2->CCR2 = *(Moter++);
	TIM2->CCR3 = *(Moter++);
	TIM2->CCR4 = *Moter;
}
void moto_STOP(void)
{
  TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;
}
