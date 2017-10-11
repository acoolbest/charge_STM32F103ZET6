/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   led应用函数接口
  ******************************************************************************
 */


#include "bsp_led.h"   

 /**
  * @brief  初始化控制LED的IO
  * @param  无
  * @retval 无
  */
void LED_GPIO_Config(void)
{		
		/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);      /*使能SWD 禁用JTAG*/

		/*开启LED的外设时钟*/
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE); 
//-----------输出类----------------
		/*设置引脚模式为通用推挽输出*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*选择要控制的GPIOA组引脚*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		/*选择要控制的GPIOB组引脚*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_14;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
		
		/*选择要控制的GPIOC组引脚*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_13;	
		GPIO_Init(GPIOC, &GPIO_InitStructure);	
		
		/*选择要控制的GPIOD组引脚*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		
		/*选择要控制的GPIOE组引脚*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_5;	
		GPIO_Init(GPIOE, &GPIO_InitStructure);	
//-----------输入类----------------
		/*设置引脚模式为通用浮空输入*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		
		/*选择要控制的GPIOA组引脚*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);	

		
		/*设置引脚模式为通用模拟输入*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;   

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		 
		/*选择要控制的GPIOA组引脚*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
				
		/*设置引脚模式为通用浮空输入*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  
		
		/*设置引脚模式为通用浮空输入*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   

		/*选择要控制的GPIOE组引脚*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
		GPIO_Init(GPIOE, &GPIO_InitStructure);	

		 
		
		/* 写1*/
//		GPIO_SetBits(GPIOB, GPIO_Pin_0);
		
		/* 写0	*/
//		GPIO_ResetBits(GPIOC, GPIO_Pin_4|GPIO_Pin_13);	 

		GPIO_SetBits(HUB0_REST_PORT, HUB0_REST_PIN);  
		GPIO_SetBits(HUB1_REST_PORT, HUB1_REST_PIN);
		
		GPIO_SetBits(EN_KC0_PORT, EN_KC0_PIN); //USB上电方式
		GPIO_SetBits(EN_KC1_PORT, EN_KC1_PIN); //USB上电方式
		
		GPIO_ResetBits(RJ45_IO1_PORT, RJ45_IO1_PIN);
		GPIO_ResetBits(EN_485_PORT, EN_485_PIN);  //默认接收
		
		GPIO_SetBits(FLASH1CS_PORT, FLASH1CS);  //默认接收

		GPIO_ResetBits(LED_PORT, LED_PIN);
		GPIO_ResetBits(LED1_PORT, LED1_PIN);
		GPIO_ResetBits(LED2_PORT, LED2_PIN);

}

//-------------------------------------------------------------------
void GPIO_NegationBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) //dwgl  取反
{
	  GPIOx->ODR = GPIOx->ODR^GPIO_Pin;
}


/*********************************************END OF FILE**********************/
