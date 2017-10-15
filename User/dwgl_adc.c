//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#include "stm32f10x.h"

u16 * ADC1_Pointer = NULL;
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

u16 * ADC3_Pointer = NULL;

u8 ADC1_channel[18]={AN1_ADC1_CH8,AN2_ADC1_CH1,AN3_ADC1_CH13,AN7_ADC1_CH15,AN8_ADC1_CH9,AN9_ADC1_CH0};
static u8 ADC1_channel_index = 0;

u8 ADC3_channel[18]={AN4_ADC3_CH6,AN5_ADC3_CH8,AN6_ADC3_CH11,AN10_ADC3_CH7,AN11_ADC3_CH10,AN12_ADC3_CH12,INPUT_AD_ADC3_CH5};
static u8 ADC3_channel_index = 0;

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
	GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC0,����ʱ������������

	/* ADC1 configuration */	
	ADC_DeInit(ADC1);   //���ȸ�λADC1��ADC��ȫ���Ĵ���Ϊȱʡֵ 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//����ADCģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 	 				//��ֹɨ��ģʽ��ɨ��ģʽ���ڶ�ͨ���ɼ�
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//��������ת��ģʽ������ͣ�ؽ���ADCת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//�ɼ������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	 								//Ҫת����ͨ����Ŀ1
	ADC_Init(ADC1, &ADC_InitStructure);

	/*����ADCʱ�ӣ�ΪPCLK2��8��Ƶ����9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	//	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
	/*����ADC1��ͨ��11Ϊ55.	5���������ڣ�����Ϊ1 */ 
	//	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/*��λУ׼�Ĵ��� */   
	ADC_ResetCalibration(ADC1);
	/*�ȴ�У׼�Ĵ�����λ��� */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* ADCУ׼ */
	ADC_StartCalibration(ADC1);
	/* �ȴ�У׼���*/
	while(ADC_GetCalibrationStatus(ADC1));
	//���ж�
	//	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//���ж�

	/* ����û�в����ⲿ����������ʹ���������ADCת�� */ 
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
	GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC0,����ʱ������������

	/* ADC1 configuration */	
	ADC_DeInit(ADC1);   //���ȸ�λADC1��ADC��ȫ���Ĵ���Ϊȱʡֵ 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//����ADCģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 	 				//��ֹɨ��ģʽ��ɨ��ģʽ���ڶ�ͨ���ɼ�
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//��������ת��ģʽ������ͣ�ؽ���ADCת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//�ɼ������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	 								//Ҫת����ͨ����Ŀ1
	ADC_Init(ADC1, &ADC_InitStructure);

	/*����ADCʱ�ӣ�ΪPCLK2��8��Ƶ����9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	//	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
	/*����ADC1��ͨ��11Ϊ55.	5���������ڣ�����Ϊ1 */ 
	//	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/*��λУ׼�Ĵ��� */   
	ADC_ResetCalibration(ADC1);
	/*�ȴ�У׼�Ĵ�����λ��� */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* ADCУ׼ */
	ADC_StartCalibration(ADC1);
	/* �ȴ�У׼���*/
	while(ADC_GetCalibrationStatus(ADC1));
	//���ж�
	//	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//���ж�

	/* ����û�в����ⲿ����������ʹ���������ADCת�� */ 
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
	GPIO_Init(GPIOC, &GPIO_InitStructure);				// PC0,����ʱ������������

	/* ADC1 configuration */	
	ADC_DeInit(ADC1);   //���ȸ�λADC1��ADC��ȫ���Ĵ���Ϊȱʡֵ 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//����ADCģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			     //��ֹɨ��ģʽ��ɨ��ģʽ���ڶ�ͨ���ɼ�
	//	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//��������ת��ģʽ������ͣ�ؽ���ADCת��
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;			//��������ת��ģʽ������ͣ�ؽ���ADCת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//�ɼ������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1; 								//Ҫת����ͨ����Ŀ1
	ADC_Init(ADC1, &ADC_InitStructure);

	/*����ADCʱ�ӣ�ΪPCLK2��8��Ƶ����9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	//	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
	/*����ADC1��ͨ��11Ϊ55.	5���������ڣ�����Ϊ1 */ 
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

	/*��λУ׼�Ĵ��� */   
	ADC_ResetCalibration(ADC1);
	/*�ȴ�У׼�Ĵ�����λ��� */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* ADCУ׼ */
	ADC_StartCalibration(ADC1);
	/* �ȴ�У׼���*/
	while(ADC_GetCalibrationStatus(ADC1));
	//���ж�
	//	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//���ж�

	/* ����û�в����ⲿ����������ʹ���������ADCת�� */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
#endif

//*******************************************
static void ADC1_Init(void)
{
	//GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* Enable ADC1*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* ADC1 configuration */	
	ADC_DeInit(ADC1);													//���ȸ�λADC1��ADC��ȫ���Ĵ���Ϊȱʡֵ 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//����ADCģʽ
	//ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//��ֹɨ��ģʽ��ɨ��ģʽ���ڶ�ͨ���ɼ�
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	//ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;				//��������ת��ģʽ������ͣ�ؽ���ADCת��
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//�ɼ������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//Ҫת����ͨ����Ŀ1
	ADC_Init(ADC1, &ADC_InitStructure);

	/*����ADCʱ�ӣ�ΪPCLK2��8��Ƶ����9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);

	/*����ADC1��ͨ��11Ϊ55.	5���������ڣ�����Ϊ1 */ 
	//	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5);

	ADC_RegularChannelConfig(ADC1, ADC1_channel[ADC1_channel_index], 1, ADC_SampleTime_239Cycles5);

	/* Enable ADC1 DMA */
	//ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/*��λУ׼�Ĵ��� */   
	ADC_ResetCalibration(ADC1);
	
	/*�ȴ�У׼�Ĵ�����λ��� */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* ADCУ׼ */
	ADC_StartCalibration(ADC1);
	
	/* �ȴ�У׼���*/
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* ���ж�*/
	//ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	/* ����û�в����ⲿ����������ʹ���������ADCת�� */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

#if 0
void ADC2_Init(void)
{

}
#endif

static void ADC3_Init(void)
{
	//GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* Enable ADC3*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	/* ADC3 configuration */	
	ADC_DeInit(ADC3);													//���ȸ�λADC3��ADC��ȫ���Ĵ���Ϊȱʡֵ 
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//����ADCģʽ
	//ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//��ֹɨ��ģʽ��ɨ��ģʽ���ڶ�ͨ���ɼ�
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	//ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;				//��������ת��ģʽ������ͣ�ؽ���ADCת��
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//�ɼ������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//Ҫת����ͨ����Ŀ1
	
	ADC_Init(ADC3, &ADC_InitStructure);

	/*����ADCʱ�ӣ�ΪPCLK2��8��Ƶ����9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	
	/*����ADC3��ͨ��11Ϊ55.	5���������ڣ�����Ϊ1 */ 
	//ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5);
	
	ADC_RegularChannelConfig(ADC3, ADC3_channel[ADC3_channel_index], 1, ADC_SampleTime_239Cycles5);

	/* Enable ADC3 DMA */
	//ADC_DMACmd(ADC3, ENABLE);

	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);

	/*��λУ׼�Ĵ��� */   
	ADC_ResetCalibration(ADC3);
	
	/*�ȴ�У׼�Ĵ�����λ��� */
	while(ADC_GetResetCalibrationStatus(ADC3));

	/* ADCУ׼ */
	ADC_StartCalibration(ADC3);
	
	/* �ȴ�У׼���*/
	while(ADC_GetCalibrationStatus(ADC3));
	
	/* ���ж�*/
	//ADC_ITConfig(ADC3, ADC_IT_EOC, ENABLE);

	/* ����û�в����ⲿ����������ʹ���������ADCת�� */
	ADC_SoftwareStartConvCmd(ADC3, ENABLE);
}

//***********************************************************************

void ADC1_2_IRQHandler(void)
{
	static u8 u8_sampling_times = 0;
	
	if(ADC_GetITStatus(ADC1, ADC_IT_EOC)!= RESET)
	{
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		
		ADC1_Pointer[ADC1_channel[ADC1_channel_index]*ADC_SAMPLING_TIMES+u8_sampling_times] = ADC_GetConversionValue(ADC1);
		
		u8_sampling_times++;
		if(u8_sampling_times >= 3)
		{
			u8_sampling_times = 0;

			ADC1_channel_index++;
			if(ADC1_channel_index >= ADC1_ENABLE_CHANNEL_NUM) ADC1_channel_index = 0;
			
			ADC_RegularChannelConfig(ADC1, ADC1_channel[ADC1_channel_index], 1, ADC_SampleTime_239Cycles5);
		}
	}
}

void ADC3_IRQHandler(void)
{
	static u8 u8_sampling_times = 0;
	
	if(ADC_GetITStatus(ADC3, ADC_IT_EOC)!= RESET)
	{
		ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);
		
		ADC3_Pointer[ADC3_channel[ADC3_channel_index]*ADC_SAMPLING_TIMES+u8_sampling_times] = ADC_GetConversionValue(ADC3);
		
		u8_sampling_times++;
		if(u8_sampling_times >= 3)
		{
			u8_sampling_times = 0;

			ADC3_channel_index++;
			if(ADC3_channel_index >= ADC3_ENABLE_CHANNEL_NUM) ADC3_channel_index = 0;
			
			ADC_RegularChannelConfig(ADC3, ADC3_channel[ADC3_channel_index], 1, ADC_SampleTime_239Cycles5);
		}
	}
}

void ADC_Configuration(void)
{
	ADC1_Pointer = ADC_BUFFER;
	ADC3_Pointer = ADC_BUFFER+ADC_BUFFER_SIZE/2;

	ADC1_Init();
	//ADC2_Init();
	ADC3_Init();
}
