/**
 ******************************************************************************
 * @file    bsp_led.c
 * @author  fire
 * @version V1.0
 * @date    2013-xx-xx
 * @brief   ledӦ�ú����ӿ�
 ******************************************************************************
 */

#include "bsp_led.h"

/**
 * @brief  ��ʼ������LED��IO
 * @param  ��
 * @retval ��
 */
void LED_GPIO_Config(void)
{		
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);      /*ʹ��SWD ����JTAG*/

	/*����LED������ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOG, ENABLE); 
	//-----------�����----------------
	/*��������ģʽΪͨ���������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

	/*������������Ϊ50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/*ѡ��Ҫ���Ƶ�GPIOA������*/
	GPIO_InitStructure.GPIO_Pin = A3_RJ45_IO_PIN|A4_SPI1_CS_PIN|A5_SPI1_CLK_PIN|A7_SPI1_MOSI_PIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*ѡ��Ҫ���Ƶ�GPIOB������*/
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_14;
	//SSC1|SSC2|BOOT1|HUB1_REST|HUB2_RESET|DB1|DB0|485_TX_EN
	GPIO_InitStructure.GPIO_Pin = B7_EN_HV4_PIN|B2_BOOT1_PIN|B8_HUB1_REST_PIN|B9_HUB2_REST_PIN|B6_L_SEL1_PIN|B5_L_SEL0_PIN|B3_R_SEL1_PIN|B4_L_SW_883_HUB_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*ѡ��Ҫ���Ƶ�GPIOC������*/
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_13;
	//SSB1|SSB0|SSB2|SSC0|LCD_RST|EN_KC1
	GPIO_InitStructure.GPIO_Pin = C9_485_TX_EN_PIN|C7_LCD_RST_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	/*ѡ��Ҫ���Ƶ�GPIOD������*/
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
	//LCD_CS1|LCD_CS2
	GPIO_InitStructure.GPIO_Pin = D12_LCD_CS1_PIN|D13_LCD_CS2_PIN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/*ѡ��Ҫ���Ƶ�GPIOE������*/
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_5;
	//DC1|DC0|LED|LED2|LED1|EN_KC0
	GPIO_InitStructure.GPIO_Pin = E0_EN_TPS54336_4_PIN|E1_EN_HV5_PIN|E2_EN_TPS54336_5_PIN|E3_EN_HV6_PIN|E4_EN_TPS54336_6_PIN|E5_EN_HV3_PIN|E6_EN_TPS54336_3_PIN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);	

	/*ѡ��Ҫ���Ƶ�GPIOEF������*/
	GPIO_InitStructure.GPIO_Pin = F0_EN_HV2_PIN|F1_EN_TPS54336_2_PIN|F2_EN_HV1_PIN|F3_EN_TPS54336_1_PIN;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/*ѡ��Ҫ���Ƶ�GPIOG������*/
	GPIO_InitStructure.GPIO_Pin = G15_R_SEL0_PIN|G14_R_SW_883_HUB_PIN|G2_L_LED_PIN|G3_R_LED_PIN|G4_LED_PIN;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	//-----------������----------------
	/*������������Ϊ50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	/*��������ģʽΪͨ�ø�������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	/*ѡ��Ҫ���Ƶ�GPIOA������*/
	GPIO_InitStructure.GPIO_Pin = A6_SPI1_MISO_PIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*��������ģʽΪͨ��ģ������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

	/*ѡ��Ҫ���Ƶ�GPIOA������*/
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	//INPUT_AD|AN02|AN01
	//F7_INPUT_AD_PIN
	//B0_AN1_PIN|A1_AN2_PIN|C3_AN3_PIN|F8_AN4_PIN|F10_AN5_PIN|C1_AN6_PIN
	//C5_AN7_PIN|B1_AN8_PIN|A0_AN9_PIN|F9_AN10_PIN|C0_AN11_PIN|C2_AN12_PIN
	GPIO_InitStructure.GPIO_Pin = A1_AN2_PIN|A0_AN9_PIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*ѡ��Ҫ���Ƶ�GPIOB������*/
	GPIO_InitStructure.GPIO_Pin = B0_AN1_PIN|B1_AN8_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*ѡ��Ҫ���Ƶ�GPIOC������*/
	GPIO_InitStructure.GPIO_Pin = C3_AN3_PIN|C1_AN6_PIN|C5_AN7_PIN|C0_AN11_PIN|C2_AN12_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*ѡ��Ҫ���Ƶ�GPIOF������*/
	GPIO_InitStructure.GPIO_Pin = F7_INPUT_AD_PIN|F8_AN4_PIN|F10_AN5_PIN|F9_AN10_PIN;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/*��������ģʽΪ��������*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

	/*ѡ��Ҫ���Ƶ�GPIOE������*/
	GPIO_InitStructure.GPIO_Pin = G13_KEY_PIN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* д1*/
	//GPIO_SetBits(GPIOB, GPIO_Pin_0);

	/* д0	*/
	//GPIO_ResetBits(GPIOC, GPIO_Pin_4|GPIO_Pin_13);

	GPIO_SetBits(HUB0_REST_PORT, HUB0_REST_PIN);
	GPIO_SetBits(HUB1_REST_PORT, HUB1_REST_PIN);

	GPIO_SetBits(EN_KC0_PORT, EN_KC0_PIN); //USB�ϵ緽ʽ
	GPIO_SetBits(EN_KC1_PORT, EN_KC1_PIN); //USB�ϵ緽ʽ

	GPIO_ResetBits(RJ45_IO1_PORT, RJ45_IO1_PIN);
	GPIO_ResetBits(EN_485_PORT, EN_485_PIN);  //Ĭ�Ͻ���

	GPIO_SetBits(FLASH1CS_PORT, FLASH1CS);  //Ĭ�Ͻ���

	led_power_ctrl(LED_INDEX, LED_TURN_OFF);
	led_power_ctrl(LED1_INDEX, LED_TURN_OFF);
	led_power_ctrl(LED2_INDEX, LED_TURN_OFF);
	
}

void GPIO_NegationBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) //dwgl  ȡ��
{
	GPIOx->ODR ^= GPIO_Pin;
}

/*********************************************END OF FILE**********************/
