/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：INIT.c
 * 描述    ：系统初始化         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "include.h"
#include "app.h"



float Battery_voltage;  
State Init_State;

void MCO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_MCOConfig(RCC_MCO_HSE); 
}

void IAC_Init(void)
{
	 delay_init(72);
	 MCO_INIT();
	 Nvic_Init(); 
   LED_GPIO_Config();	
   I2C_INIT();
	 delay(0XFFF);
	 TIM3_Init(2500);
	 usart1_config();
	 ADC1_Init();	
	 NRF24L01_Init();
   Moto_Init();	
	 LED_SHOW();
	 FLASH_Unlock();
	 EE_Init();
}

void Sensor_Init(void)
{
	Init_State.MPU6050_State = InitMPU6050();
	Init_State.NRF2401_State = NRF24L01_Check();
	NRF24L01_Mode(1);
	MS5611_init();

}

