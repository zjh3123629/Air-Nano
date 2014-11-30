/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��RC.c
 * ����    ������ң��������         
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
 * �Ա�    ��http://byd2.taobao.com
**********************************************************************************/
#include "RC.h"
#include "include.h"


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	

RC_GETDATA RC_Data;

u8 accCorrect_flag;
u8 turn_flag=0;

void RC_Analy(void)
{
	static u8 mode_chang=0,tim=0;
	static u8 turn_Read=0,turn_Cont=0;
	 if(!mode_chang)  //����ģʽ
	 {
		 NRF24L01_RxPacket(RC_Data.NRF24L01_RXDATA); 
		
		 if(RC_Data.NRF24L01_RXDATA[0] == 0xAF && RC_Data.NRF24L01_RXDATA[1] == 0x0C)  //�ж�֡ͷ
		 {
			 if(RC_Data.NRF24L01_RXDATA[2] == 0xFA)
			 {
				 RC_Data.THROTTLE = (RC_Data.NRF24L01_RXDATA[3]<<8)|RC_Data.NRF24L01_RXDATA[4];
				 
				 RC_Data.YAW      = (RC_Data.NRF24L01_RXDATA[5]<<8)|RC_Data.NRF24L01_RXDATA[6];
				 if(RC_Data.YAW > 30000)  RC_Data.YAW -= 65536;
				 
				 RC_Data.ROLL     = ((RC_Data.NRF24L01_RXDATA[7]<<8)|RC_Data.NRF24L01_RXDATA[8]);	
				 if(RC_Data.ROLL > 3000) RC_Data.ROLL -= 65536;
				 
				 RC_Data.PITCH    = ((RC_Data.NRF24L01_RXDATA[9]<<8)|RC_Data.NRF24L01_RXDATA[10]);
				 if(RC_Data.PITCH > 30000)  RC_Data.PITCH -= 65536;
				 
				 if(RC_Data.NRF24L01_RXDATA[31])  mode_chang = 1; 
				 
				 turn_Read = RC_Data.NRF24L01_RXDATA[11];
				 if(turn_Read  & (turn_Read ^ turn_Cont))    turn_flag =  1;
         turn_Cont  = turn_Read;
         //if( RC_Data.NRF24L01_RXDATA[11])  turn_flag=1;
				 
			 }
			 else if(RC_Data.NRF24L01_RXDATA[2] == 0xFB && RC_Data.NRF24L01_RXDATA[3] == 0xFB)
			 {
					accCorrect_flag = 1; 
			 }
		 }
	 }
	 if(mode_chang)   tim ++;//�ѽ��յ�֪ͨ  �л�������ģʽ
   switch(tim)
	 {
		  case 1: 
			{  
				NRF24L01_Mode(2); 
				RC_Data.NRF24L01_TXDATA[0] = 0x1A; 
				RC_Data.NRF24L01_TXDATA[1] = 0x0C;
				RC_Data.NRF24L01_TXDATA[2] = BYTE1(ADC_ConvertedValue);
				RC_Data.NRF24L01_TXDATA[3] = BYTE0(ADC_ConvertedValue);
        NRF24L01_TxPacket(RC_Data.NRF24L01_TXDATA); //�������� 
			} break;
			case 2:
			{
				NRF24L01_TxPacket(RC_Data.NRF24L01_RXDATA); //�������� 
			  NRF24L01_Mode(1);   //�л�Ϊ����ģʽ
				RC_Data.NRF24L01_RXDATA[31]=0;
				tim = mode_chang = 0;
			} break;
	 }
		 
	 
}
