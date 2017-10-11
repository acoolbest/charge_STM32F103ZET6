//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#include "stm32f10x.h"

  u16  *ADC1_Pointer;
  u16  ADC1_BufSize;
  u16  ADC1_Length ;
  u8   ADC1_State;
  u8   ADC1_Error;
  u16  ADC1_Buffer;

  u16  *ADC2_Pointer;
  u16  ADC2_BufSize;
  u16  ADC2_Length ;
  u8   ADC2_State;
  u8   ADC2_Error;
  u16  ADC2_Buffer;

void ADC_Configuration(void)
{
	ADC1_Pointer =  ADC_BUFFER;
	ADC1_BufSize = 320;
	ADC1_Length = 0;
	ADC1_Init();
//	ADC2_Init();
}
#if  defined ADC1_init_Normal
//*******************************************
void ADC1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
		
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
	
	/* Configure PC.0  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC0,输入时不用设置速率
	
	/* ADC1 configuration */	
	ADC_DeInit(ADC1);   //首先复位ADC1。ADC的全部寄存器为缺省值 
 	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 	 				//禁止扫描模式，扫描模式用于多通道采集
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	 								//要转换的通道数目1
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*配置ADC时钟，为PCLK2的8分频，即9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
//	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
	/*配置ADC1的通道11为55.	5个采样周期，序列为1 */ 
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
	
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* ADC校准 */
	ADC_StartCalibration(ADC1);
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC1));
	//开中断
//	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//开中断

	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
#endif
#if  defined ADC1_init_DMA
//*******************************************
void ADC1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
		
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
	
	/* Configure PC.0  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC0,输入时不用设置速率
	
	/* ADC1 configuration */	
	ADC_DeInit(ADC1);   //首先复位ADC1。ADC的全部寄存器为缺省值 
 	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 	 				//禁止扫描模式，扫描模式用于多通道采集
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	 								//要转换的通道数目1
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*配置ADC时钟，为PCLK2的8分频，即9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
//	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
	/*配置ADC1的通道11为55.	5个采样周期，序列为1 */ 
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
	
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* ADC校准 */
	ADC_StartCalibration(ADC1);
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC1));
	//开中断
//	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//开中断

	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
#endif
#if  defined ADC1_init_DMAE
//*******************************************
void ADC1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
		
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
	
	/* Configure PC.0  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC0,输入时不用设置速率
	
	/* ADC1 configuration */	
	ADC_DeInit(ADC1);   //首先复位ADC1。ADC的全部寄存器为缺省值 
 	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			     //禁止扫描模式，扫描模式用于多通道采集
//	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1; 								//要转换的通道数目1
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*配置ADC时钟，为PCLK2的8分频，即9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
//	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
	/*配置ADC1的通道11为55.	5个采样周期，序列为1 */ 
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5);
	ADC1_Length = 0;
	AINx_ADCch[16] = 5;
	ADC_RegularChannelConfig(ADC1, AINx_ADCch[AINx_ADCch[16]], 1, ADC_SampleTime_239Cycles5);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_239Cycles5);
// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_239Cycles5);
// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_239Cycles5);
// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 4, ADC_SampleTime_239Cycles5);
// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 5, ADC_SampleTime_239Cycles5);
// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 6, ADC_SampleTime_239Cycles5);
// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 7, ADC_SampleTime_239Cycles5);
// 	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 8, ADC_SampleTime_239Cycles5);
	
	/* Enable ADC1 DMA */
//	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* ADC校准 */
	ADC_StartCalibration(ADC1);
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC1));
	//开中断
//	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//开中断

	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
#endif
//*******************************************
void ADC2_Init(void)
{

}

//***********************************************************************
void ADC1_2_IRQHandler(void)  
{
	if(ADC_GetITStatus(ADC1, ADC_IT_EOC)!= RESET)
	{
		
			ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
			ADC1_Buffer = ADC_GetConversionValue(ADC1);
		if(ADC1_Length < ADC1_BufSize)
		{
			ADC1_Pointer[ADC1_Length] = ADC1_Buffer;
			ADC1_Length++;
			AINx_ADCch[17]++;
			if(AINx_ADCch[17]>=3)
			{			
			AINx_ADCch[16]++;
			AINx_ADCch[16] &= 0x0f;
			ADC_RegularChannelConfig(ADC1, AINx_ADCch[AINx_ADCch[16]], 1, ADC_SampleTime_239Cycles5);			
			AINx_ADCch[17]=0;
			}
			
		}	
		if(ADC1_Length>=24)
		{
			ADC1_State |= 1;
			ADC1_Length =15;
			AINx_ADCch[16] = 5;  //从第数组中的第6个开始
			AINx_ADCch[17] = 0;
			ADC_RegularChannelConfig(ADC1, AINx_ADCch[AINx_ADCch[16]], 1, ADC_SampleTime_239Cycles5);			
// 		GPIO_NegationBits(LED_PORT, LED_PIN);
	//		ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);//关中断
	//			ADC_Cmd(ADC1, DISABLE);
	//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
		}
	}
	
	if(ADC_GetITStatus(ADC2, ADC_IT_EOC)!= RESET)
	{
			ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
			ADC2_Buffer = ADC_GetConversionValue(ADC2);
	}	
	
}

