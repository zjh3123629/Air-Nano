#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"

/* the macro definition to trigger the led on or off 
 * 1 - off
 - 0 - on
 */
#define ON  0
#define OFF 1

//带参宏，可以像内联函数一样使用
#define LED1(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_2);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_2)

#define LED2(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_3);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_3)

#define LED3(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_4);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_4)
#define LED4(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_5);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_5)						
					
#define LED_ALLON()					GPIO_ResetBits(GPIOB,GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5)			
#define LED_ALLOFF()				GPIO_SetBits(GPIOB,GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5)				

void LED_GPIO_Config(void);
					void LED_SHOW(void);
void LED_Running(int tim);
void LED_Sailing(int rate);


#endif /* __LED_H */
