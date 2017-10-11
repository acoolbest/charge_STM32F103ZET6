//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
//#include "dwgl_adc.h"
#ifndef __DWGL_ADC_H
#define	__DWGL_ADC_H
#include "stm32f10x.h"

//
//#define	ADC1_init_Normal//手动更新方式
//#define	ADC1_init_DMA      // DMA方式
#define	ADC1_init_DMAE      // DMA 自动扫描
//变量
 extern  u16  *ADC1_Pointer;
 extern  u16  ADC1_BufSize;
 extern  u16  ADC1_Length ;
 extern  u8   ADC1_State;
 extern  u8   ADC1_Error;
 extern  u16  ADC1_Buffer;

 extern  u16  *ADC2_Pointer;
 extern  u16  ADC2_BufSize;
 extern  u16  ADC2_Length ;
 extern  u8   ADC2_State;
 extern  u8   ADC2_Error;
 extern  u16  ADC2_Buffer;

//子程序
extern void ADC1_Enable(void);
extern void ADC1_Disable(void);
extern void ADC1_Init(void);
extern void ADC1_Set(u8 baud,u8 work_mode);

extern void ADC2_Enable(void);
extern void ADC2_Disable(void);
extern void ADC2_Init(void);
extern void ADC2_Set(u8 baud,u8 work_mode);

extern void ADC_Configuration(void);

#endif 
