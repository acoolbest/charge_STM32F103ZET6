#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"

/** the macro definition to trigger the led on or off 
  * 1 - off
  *0 - on
  */
#define ON  0
#define OFF 1

/* 带参宏，可以像内联函数一样使用 */
#define LED1(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_0);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_0)

#define LED2(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_4);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_4)

#define LED3(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_3);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_3)


/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			//设置为高电平		
#define digitalLo(p,i)			{p->BRR=i;}				//输出低电平
#define digitalToggle(p,i)		{p->ODR ^=i;}			//输出反转状态


/* 定义控制IO的宏 */
#define LED1_TOGGLE		digitalToggle(GPIOB,GPIO_Pin_0)
#define LED1_OFF		digitalLo(LED1_PORT,LED1_PIN)
#define LED1_ON			digitalHi(LED1_PORT,LED1_PIN)
//MOS ON/OFF
#define SSB0_PIN			  GPIO_Pin_3
#define SSB0_PORT			  GPIOC
#define SSB1_PIN			  GPIO_Pin_2
#define SSB1_PORT			  GPIOC
#define SSB2_PIN			  GPIO_Pin_4
#define SSB2_PORT			  GPIOC
#define SSC0_PIN			  GPIO_Pin_5
#define SSC0_PORT			  GPIOC
#define SSC1_PIN			  GPIO_Pin_0
#define SSC1_PORT			  GPIOB
#define SSC2_PIN			  GPIO_Pin_1
#define SSC2_PORT			  GPIOB

#define AIN0_PIN			  GPIO_Pin_0   //输入电压监测
#define AIN0_PORT			  GPIOA
#define AIN1_PIN			  GPIO_Pin_1
#define AIN1_PORT			  GPIOA
#define AIN2_PIN			  GPIO_Pin_2
#define AIN2_PORT			  GPIOA


#define EN_485_PIN			GPIO_Pin_14
#define EN_485_PORT			GPIOB

#define LED_PIN			    GPIO_Pin_3
#define LED_PORT			  GPIOE
#define LED1_PIN			  GPIO_Pin_6
#define LED1_PORT			  GPIOE
#define LED2_PIN			  GPIO_Pin_4
#define LED2_PORT			  GPIOE
#define KEY_PIN			    GPIO_Pin_2
#define KEY_PORT			  GPIOE

#define RJ45_IO1_PIN			GPIO_Pin_3  
#define RJ45_IO1_PORT			GPIOA

#define USB_DB0_PIN			  GPIO_Pin_9   //USB,3to1
#define USB_DB0_PORT			GPIOB
#define USB_DB1_PIN			  GPIO_Pin_8
#define USB_DB1_PORT			GPIOB
#define USB_DC0_PIN			  GPIO_Pin_1
#define USB_DC0_PORT			GPIOE
#define USB_DC1_PIN			  GPIO_Pin_0
#define USB_DC1_PORT			GPIOE

#define DPB1_PIN			   GPIO_Pin_12   //本地USB口
#define DPB1_PORT			   GPIOA
#define DPC3_PIN			   GPIO_Pin_11
#define DPC3_PORT			   GPIOA

#define SPI2_MOSI_PIN			GPIO_Pin_7
#define SPI2_MOSI_PORT		GPIOA
#define SPI2_MISO_PIN			GPIO_Pin_6
#define SPI2_MISO_PORT		GPIOA
#define SPI2_CLK_PIN			GPIO_Pin_5
#define SPI2_CLK_PORT			GPIOA
#define SPI2_CS_PIN			  GPIO_Pin_4
#define SPI2_CS_PORT			GPIOA

#define LCD_CS1_PIN			  GPIO_Pin_12
#define LCD_CS1_PORT			GPIOD
#define LCD_CS2_PIN			  GPIO_Pin_13
#define LCD_CS2_PORT			GPIOD
#define LCD_RST_PIN			  GPIO_Pin_7
#define LCD_RST_PORT			GPIOC
// #define LCD_TE_PIN			  GPIO_Pin_8  //功能已取消
// #define LCD_TE_PORT			  GPIOC
// #define LCD_RS_PIN			  GPIO_Pin_6
// #define LCD_RS_PORT			  GPIOC
// #define LCD_WR_PIN			  GPIO_Pin_2
// #define LCD_WR_PORT			  GPIOD
// #define LCD_RD_PIN			  GPIO_Pin_12
// #define LCD_RD_PORT			  GPIOC

// #define EN_KC0_PIN			 GPIO_Pin_13    //快充切换
// #define EN_KC0_PORT			 GPIOC
// #define EN_KC1_PIN			 GPIO_Pin_5
// #define EN_KC1_PORT			 GPIOE

#define EN_KC1_PIN			 GPIO_Pin_13    //快充切换
#define EN_KC1_PORT			 GPIOC
#define EN_KC0_PIN			 GPIO_Pin_5
#define EN_KC0_PORT			 GPIOE

#define HUB0_REST_PIN			 GPIO_Pin_5
#define HUB0_REST_PORT		 GPIOB
#define HUB1_REST_PIN			 GPIO_Pin_6
#define HUB1_REST_PORT		 GPIOB

void LED_GPIO_Config(void);
void GPIO_NegationBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); //dwgl  取反


#endif /* __LED_H */
