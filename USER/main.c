/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��main.c
 * ����    ��ϵͳ��ʼ��         
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
 * �Ա�    ��http://byd2.taobao.com
**********************************************************************************/
#include "include.h"

extern u8  sentDateFlag;

int main(void)
{
	  SystemInit();  //����ϵͳʱ��
	  IAC_Init();
	  Sensor_Init();
	  paramLoad();  //���ز���
	  EnTIM3();     //����ʱ�ж�
	  while(1)
		{
			if(sentDateFlag)  //10MS����λ������һ������
			{
				 sentDateFlag = 0;
         UART1_ReportIMU();  //���ڷ�����̬
				
			}
		}
}
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/

 
