#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"

void SysTick_Init(void);
//void Delay_us(__IO u32 nTime);			// ��λ1us
//#define Delay_ms(x) Delay_us(1000*x)		//��λms
//#define Delay_ms(x) Delay_us(100*x)		//��λmsdwgl
void Delay_ms(__IO u32 nTime);				//add by zhzq @ 2017-10-11 14:03:56

#endif /* __SYSTICK_H */
