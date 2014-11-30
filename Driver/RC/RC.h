#ifndef _RC_H_
#define _RC_H_
#include "stm32f10x.h"
typedef struct {
	      u8  NRF24L01_RXDATA[33];
	      u8  NRF24L01_TXDATA[33];
	      u8  ARMED;
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
	      int16_t pitch_offset;
	      int16_t roll_offset;
	      int16_t yaw_offset;
				int16_t AUX1;}RC_GETDATA;


extern  RC_GETDATA RC_Data;
extern  u8 accCorrect_flag;
extern  u8 turn_flag; 
void RC_Analy(void);
#endif
