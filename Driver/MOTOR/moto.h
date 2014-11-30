#ifndef _MOTO_H_
#define _MOTO_H_
#include "stm32f10x.h"

#define Moto_PwmMax 3500


void Moto_Init(void);
void moto_STOP(void);
void moto_PwmRflash(int16_t *Moter);
#endif
