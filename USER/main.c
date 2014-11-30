/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：main.c
 * 描述    ：系统初始化         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "include.h"

extern u8  sentDateFlag;

int main(void)
{
	  SystemInit();  //配置系统时钟
	  IAC_Init();
	  Sensor_Init();
	  paramLoad();  //加载参数
	  EnTIM3();     //开定时中断
	  while(1)
		{
			if(sentDateFlag)  //10MS向上位机发送一次数据
			{
				 sentDateFlag = 0;
         UART1_ReportIMU();  //串口发送姿态
				
			}
		}
}
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/

 
