/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   Һ����������ʵ��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� ISO-MINI STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
 
#include "stm32f10x.h"
// //void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
// {
// 	for(; nCount != 0; nCount--);
// }

/**
  * @brief  ������
  * @param  ��  
  * @retval ��
  */
int main(void)
{	
	u8 flag;
	u32 i;
	/* ϵͳ��ʱ�� 1us ��ʱ��ʼ�� */
  SysTick_Init();
//	CPU_CRITICAL_ENTER();
  LED_GPIO_Config();	
	ADC_Configuration();
//	DAC_Configuration();
	UART_Configuration();
	SPI_Configuration();
	DMA_Configuration();
	LCD_IOConfig();
	FSMC_LCDInit();	
	
	NVIC_Configuration();
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//���ж�
//	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
//	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);

//	CPU_CRITICAL_EXIT();
	device.Version[0] = 17; //2017��
	device.Version[1] = 41; //4�µ�1��
	step =0;
	time_s = 0;
	time_sys = 0;
	check_time = time_sys;
	device.TASK_state =0x00;
//	ADC_Cmd(ADC1, ENABLE);
		Delay_us(150);	
		GPIO_NegationBits(LED_PORT, LED_PIN);
		Delay_us(150);	
		GPIO_NegationBits(LED_PORT, LED_PIN);
		Delay_us(150);	
		GPIO_NegationBits(LED_PORT, LED_PIN);
		Delay_us(150);	
		GPIO_NegationBits(LED_PORT, LED_PIN);
		Delay_us(150);	
		GPIO_NegationBits(LED_PORT, LED_PIN);
		Delay_us(150);	
		GPIO_NegationBits(LED_PORT, LED_PIN);
		
		FLASH2_GPIOSPI_Read (Addr_01min, str_buffer, 128);
		if(str_buffer[0]==frame_headerC)  //�ж���û�ж�ά��
		{
			device.use &= ~0x0C; //�ж�ά�� 
		}
		else
		{
			device.use |= 0x0C;//û�ж�ά��
		}
			
				
// 		Addr_Set();
//		hub_id_info();
//��ʼ���ϵ�MOS���ر��ϵ�
		GPIO_SetBits(SSB0_PORT, SSB0_PIN);  
		GPIO_SetBits(SSB1_PORT, SSB1_PIN);
		GPIO_SetBits(SSB2_PORT, SSB2_PIN);
		GPIO_SetBits(SSC0_PORT, SSC0_PIN);
		GPIO_SetBits(SSC1_PORT, SSC1_PIN);
		GPIO_SetBits(SSC2_PORT, SSC2_PIN);
				
//��ʼLCD
 	GPIO_ResetBits(LCD_RST_PORT, LCD_RST_PIN);
		LCD_Init();	
		LCD_Init1();	
 	GPIO_ResetBits(LCD_CS1_PORT, LCD_CS1_PIN);
 	GPIO_ResetBits(LCD_CS2_PORT, LCD_CS2_PIN);
		LCD_Clear(RED);
		LCD_Clear(GREEN);
		LCD_Clear(BLUE);
//		tft_16bitdeep_BMP (220,0,gImage11_240_100_16bitC);		//ԭLOGO
 	GPIO_SetBits(LCD_CS1_PORT, LCD_CS1_PIN);
 	GPIO_SetBits(LCD_CS2_PORT, LCD_CS2_PIN);
	
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
			
			LCDC.LCD1POFFTime = 0XFF;
			LCDC.LCD2POFFTime = 0XFF;
			LCDC.PSwitch = 1;
			LCDC.SPSwitch = 1;
			LCDC.PNum = 0;
			LCDC.LCDPTimeSet = 15;  
			LCDC.LCD1PTime = 0;
			LCDC.LCD2PTime = 0;
			LCDC.LCD1PID =0;
			LCDC.LCD2PID =0;
			LCDC.LCDSPTimeSet = 60;   //����
			LCDC.LCD1SPTime = 0;
			LCDC.LCD2SPTime = 0;
			LCDC.LCD1SPPID =0;
			LCDC.LCD2SPPID =0;
		display_flash_BMPE (0,0,3,LCDC.LCD1SPPID,3);//��ɫ��ɫ��֧�� ������
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
			
		FLASH2_GPIOSPI_Read (Addr_04min, str_buffer, 64);  //��ȡͼƬ����
		if(str_buffer[0] == 0x67)//����Ч
		{
		LCDC.PNum = str_buffer[1];
		}
		FLASH2_GPIOSPI_Read (Addr_info2, &info2STR.head[0], sizeof(info2STR));  //ý�ݳ�ʼ��
		if(info2STR.head[0] == 0x67 && info2STR.item1[1]==3)//����Ч
		{
			LCDC.PSwitch = info2STR.item1_data[0];
			LCDC.LCDPTimeSet = info2STR.item1_data[1];	
			LCDC.LCDPTimeSet <<= 8;	
			LCDC.LCDPTimeSet += info2STR.item1_data[2];	
		}
		
		if(info2STR.head[0] == 0x67 && info2STR.item2[1]==3)//����Ч
		{
			LCDC.SPSwitch = info2STR.item2_data[0];
			LCDC.LCDSPTimeSet = info2STR.item2_data[1];	
			LCDC.LCDSPTimeSet <<= 8;	
			LCDC.LCDSPTimeSet += info2STR.item2_data[2];	
		}		
				
//��ʼʱ��
		Uport_PowerUseTime[0]= 0;		
		Uport_PowerUseTime[1]= Uport_PowerUseTime[0];
		Uport_PowerShowTime[0]=Uport_PowerUseTime[0];
		Uport_PowerShowTime[1]=Uport_PowerUseTime[1];
// 		tft_DisplayStr(52, 0, AnsiChardata,POINT_COLOR, BACK_COLOR,3);

//  		display_flash_BMPE (14,118,3,((Uport_PowerUseTime[0]%3600/600)+'0'),3);//ʱ�� ��H
//  		display_flash_BMPE (14,139,3,((Uport_PowerUseTime[0]%600/60)+'0'),3);//ʱ�� ��L
//  		display_flash_BMPE (14,174,3,((Uport_PowerUseTime[0]%60/10)+'0'),3);//ʱ�� ��H
//  		display_flash_BMPE (14,195,3,((Uport_PowerUseTime[0]%10)+'0'),3);//ʱ�� ��L

		
//��ȡ����,�豸��
//Addr_info,��ʽ��0X67 LEN ADDR F0 XX XX (16�ֽ�AD����) (12�ֽ��豸��) CHECK 0X99
		FLASH2_GPIOSPI_Read (Addr_info, str_buffer, 64);
		global_u8p = (u8*)ADC_Base0;
		for(i=0;i<16;i++)
		{
				global_u8p[i] = str_buffer[6+i];
//				global_u8p[i] = str_buffer[i];
		}
//  		DisplayADC_BL(150, 0, ADC_Base0,POINT_COLOR, BACK_COLOR,1);
//  		DisplayADC_BL(150, 0, &ADC_Base0[3],POINT_COLOR, BACK_COLOR,2);
//  		DisplayADC_BL(150, 0, &ADC_Base0[5],POINT_COLOR, BACK_COLOR,3);
		
		FLASH2_GPIOSPI_Read (Addr_info1, str_buffer, 64);
		if(str_buffer[0]==frame_headerC)
		{
			for(i=0;i<12;i++)
			{
				device_num[i]= str_buffer[5+i];
			}
				device_num[i++]= ' ';
				device_num[i++]= 0;
		}
		else
		{
			for(i=0;i<12;i++)
			{
				device_num[i]= '?';
			}
				device_num[i++]= ' ';
				device_num[i++]= 0;
		}
		tft_DisplayStr(270, 125, device_num,0x0000,0xffff,3);
	//	tft_DisplayStr(290, 125, device_num,0x0000,0xffff,3);

//�ı�����ʼ		
	//	FiletoBuffer_ID(u8 area,u8 id,u8 *p);
		FiletoBuffer_ID(2,48,LCD1_TxtBuffer);
		FiletoBuffer_ID(2,48,LCD2_TxtBuffer);
		LCD1_TxtBuffer[2048]=0;
		LCD1_TxtBuffer[2049]=0;
		LCD2_TxtBuffer[2048]=0;
		LCD2_TxtBuffer[2049]=0;
//��ά���ʼ		
			if((device.use&0x04)==0x0) //������ʾ
			{
				DisplayPROT_EWM(80,56,0,1);  //128
				DisplayPROT_EWM(80,56,1,2);  //128
//				DisplayPROT_EWM(126,40,1,2);  //160
			}
		Delay_us(100);	
	time_sys = 0;
	while(1)
	{
		if(GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN) ==0)  //����=0,1S������Ϊ��λ
		{
	//		SystemReset();
			
		GPIO_SetBits(SSB0_PORT, SSB0_PIN);
		GPIO_SetBits(SSB1_PORT, SSB1_PIN);
		GPIO_SetBits(SSB2_PORT, SSB2_PIN);
		GPIO_SetBits(SSC0_PORT, SSC0_PIN);
		GPIO_SetBits(SSC1_PORT, SSC1_PIN);
		GPIO_SetBits(SSC2_PORT, SSC2_PIN);
			Delay_us(200);	
		//��ȡ����
		Get_ADC_BaseLine();
		FLASH2_GPIOSPI_Read (Addr_info, str_buffer, 64);
		global_u8p = (u8*)ADC_Base0;
		for(i=0;i<16;i++)
		{
			str_buffer[6+i] = global_u8p[i];
		}
		str_buffer[0] = 0x67;
		str_buffer[1] = 0x16;
		str_buffer[2] = device.addr;
		str_buffer[3] = 0xf0;
		str_buffer[(str_buffer[1]<<1)-2] = 0;//check;
		for(i=1;i<((str_buffer[1]<<1)-2);i++)
		{
			str_buffer[(str_buffer[1]<<1)-2] += str_buffer[i];
		}
		str_buffer[(str_buffer[1]<<1)-1] = 0x99;
		
		FLASH2_GPIOSPI_SER(Addr_info);  ////ÿ�β���4K
		FLASH2_GPIOSPI_Read (Addr_info, &str_buffer[200], 64);
		FLASH2_GPIOSPI_Write(Addr_info, str_buffer, (str_buffer[1]<<1));
		FLASH2_GPIOSPI_Read (Addr_info, str_buffer, 64);

// 			i=100000;
// 			while(i--)
// 			{
// 				GPIO_SetBits(LED_PORT, LED_PIN);
// 			}
			
			
			NVIC_SystemReset();
		}		
		
		uart1_cmd();			
		uart3_cmd();		
		
    if(time_sys-check_time >= 5000)			//��ʱ����豸
		{
			check_time += 5000;
//			GPIO_NegationBits(LED_PORT, LED_PIN);
			GPIO_SetBits(LED_PORT, LED_PIN);
			
			if((device.use&0x10)==0x10) //��ά���и���
			{
				device.use &= ~0x10; //
				DisplayPROT_EWM(80,56,0,1);  //128
				DisplayPROT_EWM(80,56,1,2);  //128
			}
		}
		
    if(UART1_Error==1)			//������
		{			
			UART1_Error = 0;
		}
		
    if(UART2_Error==1)			//������
		{	
			
		}
		
		uart1_cmd();			
		uart3_cmd();		
		
    if(time_sys-time_s_temp >= 1000)			//ʱ���������
		{
			time_s_temp +=1000;
			time_s++;		
			if(LCDC.LCD1POFFTime<0xff)	//�ϵ��ʱ
			{
			LCDC.LCD1POFFTime++;
			}
			if(LCDC.LCD2POFFTime<0xff)	//�ϵ��ʱ
			{
			LCDC.LCD2POFFTime++;
			}
			LCDC.LCD1PTime++;		//����ʱ
			LCDC.LCD2PTime++;
			LCDC.LCD1SPTime++;  //����ʱ��
			LCDC.LCD2SPTime++;  //����ʱ��
// 			if((LCDC.LCD1SPTime-LCDC.LCD2SPTime)<3)  //Ϊ������ʱ��
// 			{
// 				LCDC.LCD1SPTime++;
// 			}
//			Dport_ChargeState();
			if((checking_portB&0xF0)==0x40) 
			{
			ChargeCtrl_B();	//�����ϵ�
			}
			if((checking_portC&0xF0)==0x40) 
			{
			ChargeCtrl_C();	//�����ϵ�
			}
			UART_BUFFER[0] ='1';//����1����
			for(i=0;i<3;i++)
				{
					UART_BUFFER[1+i] = Dport_State[i]+'0';
				}
				UART_BUFFER[1+i] =0;
//			if((LCDC.LCD1SPPID!=2))	
			{
 			tft_DisplayStr(0, 32, UART_BUFFER,POINT_COLOR, BACK_COLOR,1);
			}
			UART_BUFFER[0] ='2';  //����2����
			for(i=0;i<3;i++)
				{
					UART_BUFFER[1+i] = Dport_State[3+i]+'0';
				}
				UART_BUFFER[1+i] =0;
//			if((LCDC.LCD2SPPID!=2))	
			{
 			tft_DisplayStr(0, 32, UART_BUFFER,POINT_COLOR, BACK_COLOR,2);
			}
			
			AnsiChardata[12] = Uport_PowerUseTime[0]/36000+'0';
			AnsiChardata[13] = Uport_PowerUseTime[0]%36000/3600+'0';
			AnsiChardata[19] = Uport_PowerUseTime[0]%3600/600+'0';
			AnsiChardata[20] = Uport_PowerUseTime[0]%600/60+'0';
			AnsiChardata[24] = Uport_PowerUseTime[0]%60/10+'0';
			AnsiChardata[25] = Uport_PowerUseTime[0]%10+'0';
				
		if(LCDC.LCD1SPPID==2)
		{
 		display_flash_BMPE (18,120,3,((Uport_PowerShowTime[0]%3600/600)+'0'),1);//ʱ�� ��H
 		display_flash_BMPE (18,141,3,((Uport_PowerShowTime[0]%600/60)+'0'),1);//ʱ�� ��L
 		display_flash_BMPE (18,176,3,((Uport_PowerShowTime[0]%60/10)+'0'),1);//ʱ�� ��H
 		display_flash_BMPE (18,197,3,((Uport_PowerShowTime[0]%10)+'0'),1);//ʱ�� ��L
		}
				
			if(Uport_PowerUseTime[0]>0)
			{
			Uport_PowerUseTime[0]--;
//			GPIO_SetBits(LED1_PORT, LED1_PIN);
			LCDC.LCD1POFFTime=0; //�ϵ��ʱ
			}
			else
			{
//			GPIO_ResetBits(LED1_PORT, LED1_PIN);
			Dport_ChargeOFF(0);
			Dport_ChargeOFF(1);
			Dport_ChargeOFF(2);
			Dport_State[0] = 0;
			Dport_State[1] = 0;
			Dport_State[2] = 0;
			LCDC.LCD1PTime=0;		//����ʱ
			}

			if(Uport_PowerShowTime[0]>0)
			{
			Uport_PowerShowTime[0]--;
			GPIO_SetBits(LED1_PORT, LED1_PIN);
			}
			else
			{
			GPIO_ResetBits(LED1_PORT, LED1_PIN);
			}
			
			if(LCDC.LCD1POFFTime==1)  //�ϵ�1�������
			{
			FiletoBuffer_ID(2,48,LCD1_TxtBuffer);
			LCD1_TxtBuffer[2048]=0;
			LCD1_TxtBuffer[2049]=0;
			}
			else
			if(LCDC.LCD1POFFTime==5)  //�ϵ��5�������
			{
				LCDC.LCD1SPTime = 0;  //����ʱ��
				LCDC.LCD1SPPID = 0;
				display_flash_BMPE (0,0,3,LCDC.LCD1SPPID,1);//��ɫ��ɫ��֧�� ������
				DisplayPROT_EWM(80,56,0,1);  //128
				tft_DisplayStr(270, 125, device_num,0x0000,0xffff,1);
			}
			
			AnsiChardata[12] = Uport_PowerUseTime[1]/36000+'0';
			AnsiChardata[13] = Uport_PowerUseTime[1]%36000/3600+'0';
			AnsiChardata[19] = Uport_PowerUseTime[1]%3600/600+'0';
			AnsiChardata[20] = Uport_PowerUseTime[1]%600/60+'0';
			AnsiChardata[24] = Uport_PowerUseTime[1]%60/10+'0';
			AnsiChardata[25] = Uport_PowerUseTime[1]%10+'0';

		if(LCDC.LCD2SPPID==2)
		{
		display_flash_BMPE (18,120,3,((Uport_PowerShowTime[1]%3600/600)+'0'),2);//ʱ�� ��H
 		display_flash_BMPE (18,141,3,((Uport_PowerShowTime[1]%600/60)+'0'),2);//ʱ�� ��L
 		display_flash_BMPE (18,176,3,((Uport_PowerShowTime[1]%60/10)+'0'),2);//ʱ�� ��H
 		display_flash_BMPE (18,197,3,((Uport_PowerShowTime[1]%10)+'0'),2);//ʱ�� ��L
		}
		
		
			if(Uport_PowerUseTime[1]>0)
			{				
//			GPIO_SetBits(LED2_PORT, LED2_PIN);
			Uport_PowerUseTime[1]--;
			LCDC.LCD2POFFTime=0;     //�ϵ��ʱ
			}
			else
			{
//			GPIO_ResetBits(LED2_PORT, LED2_PIN);
			Dport_ChargeOFF(3);
			Dport_ChargeOFF(4);
			Dport_ChargeOFF(5);
			Dport_State[3] = 0;
			Dport_State[4] = 0;
			Dport_State[5] = 0;
			LCDC.LCD2PTime=0;		//����ʱ
			}
			
			if(Uport_PowerShowTime[1]>0)
			{
			Uport_PowerShowTime[1]--;
			GPIO_SetBits(LED2_PORT, LED2_PIN);
			}
			else
			{
			GPIO_ResetBits(LED2_PORT, LED2_PIN);
			}

			if(LCDC.LCD2POFFTime==1)  //�ϵ��һ�������
			{
			FiletoBuffer_ID(2,48,LCD2_TxtBuffer);
			LCD2_TxtBuffer[2048]=0;
			LCD2_TxtBuffer[2049]=0;
			}
			else
			if(LCDC.LCD2POFFTime==5)  //�ϵ��5�������
			{
				LCDC.LCD2SPTime = 0;  //����ʱ��
				LCDC.LCD2SPPID = 0;
				display_flash_BMPE (0,0,3,LCDC.LCD2SPPID,2);//��ɫ��ɫ��֧�� ������
				DisplayPROT_EWM(80,56,1,2);  //128
				tft_DisplayStr(270, 125, device_num,0x0000,0xffff,2);
			}
		}
		
		uart1_cmd();			
		uart3_cmd();		
		
    if((time_sys-time_sys_temp1 >= 1000))			//ʱ���������
		{
			time_sys_temp1 +=1000;
			if(LCDC.LCD1SPPID==2)
			{
//			tft_1bitdeep_TXT (18, 0, LCD1_TxtBuffer,POINT_COLOR, BACK_COLOR,1);
			tft_1bitdeep_TXT (87, 0, LCD1_TxtBuffer,POINT_COLOR, 0xffff,1);
			}
		}
		
		uart1_cmd();			
		uart3_cmd();		
		
    if((time_sys-time_sys_temp2 >= 1000))			//ʱ���������
		{
			time_sys_temp2 +=1000;
			if(LCDC.LCD2SPPID==2)
			{
//			tft_1bitdeep_TXT (18, 0, LCD2_TxtBuffer,POINT_COLOR, BACK_COLOR,2);
			tft_1bitdeep_TXT (87, 0, LCD2_TxtBuffer,POINT_COLOR, 0xffff,2);
			}
		}
		
		uart1_cmd();			
		uart3_cmd();
		
    if(time_sys-time_sys_temp3 >= 2000)			//ʱ���������
		{
			time_sys_temp3 +=2000;
			if((time_s-testcmd3_time)>=5)  //0x16 0xe2����ȷ�����ӹ���
			{
				UART_BUFFER[0] = 'U';
// 				UART_BUFFER[1] = 'n';
// 				UART_BUFFER[2] = 'l';
// 				UART_BUFFER[3] = 'i';
// 				UART_BUFFER[4] = 0;
				UART_BUFFER[1] = (device.addr>>4)+'0';
				UART_BUFFER[2] = (device.addr&0x0f)+'0';
				UART_BUFFER[3] = 0;
				if((time_s-testcmd1_time)>=30)  //30��û�յ���ȷ������Ϊ�Ͽ�����
				{
				UART_BUFFER[0] = 'u';
				}
				tft_DisplayStr(0, 0, UART_BUFFER,POINT_COLOR, BACK_COLOR,3);
				GPIO_ResetBits(RJ45_IO1_PORT, RJ45_IO1_PIN);
			}
			else
			{
				UART_BUFFER[0] = 'L';
// 				UART_BUFFER[1] = 'i';
// 				UART_BUFFER[2] = 'n';
// 				UART_BUFFER[3] = 'k';
// 				UART_BUFFER[4] = 0;
				UART_BUFFER[1] = (device.addr>>4)+'0';
				UART_BUFFER[2] = (device.addr&0x0f)+'0';
				UART_BUFFER[3] = 0;
				if((time_s-testcmd1_time)>=30)  //30��û�յ���ȷ������Ϊ�Ͽ�����
				{
				UART_BUFFER[0] = 'l';
				}
				tft_DisplayStr(0, 0, UART_BUFFER,POINT_COLOR, BACK_COLOR,3);
			}
		}
		
		if(Uport_PowerUseTime[0]>0)
		{
			Dport_ChargeStateB();
		}
		else
		{
		checking_portB = 0x00;  //SUB�ÿ�
		}
		
		if(Uport_PowerUseTime[1]>0)
		{
			Dport_ChargeStateC();
		}
		else
		{
		checking_portC = 0x00;  //SUB�ÿ�
		}
		
		if((LCDC.LCD1SPTime>=LCDC.LCDSPTimeSet)&&(time_sys-time_s_temp<300))//LCD1��������
		{
				LCDC.LCD1SPTime -=LCDC.LCDSPTimeSet;
			  if(LCDC.LCD1SPPID!=2)  //
				{
					if(LCDC.LCD1SPPID<1)
					{
						LCDC.LCD1SPPID++;
					}
					else
					{
						LCDC.LCD1SPPID = 0;
					}
					display_flash_BMPE (0,0,3,LCDC.LCD1SPPID,1);//��ɫ��ɫ��֧�� ������
					if(LCDC.LCD1SPPID==1)
					{
					DisplayPROT_EWM(115,56,0,1);  //128
					tft_DisplayStr(15, 125, device_num,0x0000,0xffff,1);
					}
					else
					{
					DisplayPROT_EWM(80,56,0,1);  //128
					tft_DisplayStr(270, 125, device_num,0x0000,0xffff,1);
					}
				}	
		}
		
		uart1_cmd();			
		uart3_cmd();
		
		if((LCDC.LCD2SPTime>=LCDC.LCDSPTimeSet)&&(time_sys-time_s_temp<300))//LCD2��������
		{
				LCDC.LCD2SPTime -=LCDC.LCDSPTimeSet;
			  if(LCDC.LCD2SPPID!=2)
				{
					if(LCDC.LCD2SPPID<1)
					{
						LCDC.LCD2SPPID++;
					}
					else
					{
						LCDC.LCD2SPPID = 0;
					}
					display_flash_BMPE (0,0,3,LCDC.LCD2SPPID,2);//��ɫ��ɫ��֧�� ������
					if(LCDC.LCD2SPPID==1)
					{
					DisplayPROT_EWM(115,56,1,2);  //128
					tft_DisplayStr(15, 125, device_num,0x0000,0xffff,2);
					}
					else
					{
					DisplayPROT_EWM(80,56,1,2);  //128
					tft_DisplayStr(270, 125, device_num,0x0000,0xffff,2);
				}
			 }
		}

		if((LCDC.LCD1PTime>=LCDC.LCDPTimeSet)&&(time_sys-time_s_temp<300))//LCD1���¹��
		{
			LCDC.LCD1PTime -=LCDC.LCDPTimeSet;
			
			if(LCDC.PSwitch==1)
			{
					LCDC.LCD1PID++;
				if(LCDC.LCD1PID>=LCDC.PNum)
				{
					LCDC.LCD1PID=0;
				}
				display_flash_BMPE (116,0,4,LCDC.LCD1PID,1);//���1
			}
		}
		if((LCDC.LCD2PTime>=LCDC.LCDPTimeSet)&&(time_sys-time_s_temp<300))//LCD2���¹��
		{
			LCDC.LCD2PTime -=LCDC.LCDPTimeSet;
			if(LCDC.PSwitch==1)
			{
					LCDC.LCD2PID++;
				if(LCDC.LCD2PID>=LCDC.PNum)
				{
					LCDC.LCD2PID=0;
				}
				display_flash_BMPE (116,0,4,LCDC.LCD2PID,2);//���2
			}
		}
		
		uart1_cmd();			
		uart3_cmd();
		
		if((Uport_PowerUseTime[0]>0)&&(LCDC.LCD1SPPID!=2))//LCD1������
		{
			LCDC.LCD1SPPID = 2;
			display_flash_BMPE (0,0,3,LCDC.LCD1SPPID,1);//��ɫ��ɫ��֧�� ������
//			display_flash_BMPE (117,0,4,LCD1_CTRL8[5],1);//���1
		}
		
		if((Uport_PowerUseTime[1]>0)&&(LCDC.LCD2SPPID!=2))//LCD2������
		{
			LCDC.LCD2SPPID = 2;
			display_flash_BMPE (0,0,3,LCDC.LCD2SPPID,2);//��ɫ��ɫ��֧�� ������
//			display_flash_BMPE (117,0,4,LCD2_CTRL8[5],2);//���2
		}
		
	}
  

 }

void uart1_cmd (void)
{
		u16 last_i;
		u8 last_d;
	UART1_Receive_Length = UART1_RXBUFFE_LAST - UART1_RXBUFFE_HEAD;
	UART1_Receive_Length &= UART1_RX_MAX;//����ֽ�
/********������ַ���Ϊһ֡���ݵĽ�����ʶ************/
		if(UART1_Receive_Length > 0 )	//ֻ�н��յ�1���������ϲ����ж�
		{
			if(UART1_RXBUFFER[UART1_RXBUFFE_HEAD] == frame_headerC) 	//֡��ʼ��־   
			{
				UART1_RX_State = 1;
				if((UART1_Receive_Length >= 2)&&((UART1_Receive_Pointer[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX]<<1) <= UART1_Receive_Length)) 	//���ȸպñ�־   
				{
					UART1_RX_State |= 2;
					last_i = UART1_RXBUFFE_HEAD+(UART1_Receive_Pointer[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX]<<1)-1;
					last_i &= UART1_RX_MAX;
					last_d = UART1_Receive_Pointer[last_i];
					if((last_d == frame_last))//У�����
					{
						testcmd1_time = time_s;
						if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX] ==device.addr) || (UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX] ==GLOBLE_ADDR)) 
						{			
							switch(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX])
							{    		
		//					case 0x51:  cmd_Device_Info();		    break;//��ȡ�豸��Ϣ
							case 0x53:  cmd_Port_Info();		    		break;//��ȡ�˿���Ϣ
							case 0x55:  cmd_Device_Check();		    	break;//�˶���Ϣ
							case 0x57:  cmd_Device_num();		    		break;//�豸��
							case 0x59:  cmd_Power_off();		    		break;//�ϵ�����
							case 0x5a:  cmd_Power_on();		    			break;//�ϵ�����
								
//		          case 0x10:  cmd_Hub_Rst();						  break;//��λHUB		 
							case 0x11:  cmd_File_Requst();		    	break;//�ļ���������
							case 0x12:  cmd_File_Tx();		    			break;//�ļ�����
							case 0x13:  cmd_File_Recall();		    	break;//�ļ�����
							case 0x14:  cmd_File_Erase();		    		break;//�ļ�����
							case 0x16:  cmd_ShakeHands();		    break;//����
							case 0x30:  cmd_MediaCtrl();   				break;//ý���������
							case 0x3E:  Device_Rst();   				break;//��λ�豸
								
		//          case 0xE1:  cmd_Get_State();   			break;//��HUB�ŵ�
		//          case 0xE2:  cmd_Set_State();   			break;//����HUB�ŵ�FLASH
							case 0xE3:  cmd_Erase_Flash();      	break;//
							case 0xE4:  cmd_Read_Flash();       	break;//
							case 0xE5:  cmd_Write_Flash();      	break;//						
							case 0xE6:  cmd_Get_ADC();      			break;//						
							case 0xE7:  cmd_Save_ADC();   				break;//����ADC����
							case 0xE8:  cmd_RGB888_565();   			break;//
							case 0xE9:  cmd_RGB_clear();   			  break;//

							default:	    break;		   	
							}//end switch
						}
						else
						if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX] ==Broadcast)) 
						{
							switch(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX])
							{    		
		//          case 0x10:  cmd_Hub_Rst();		    	break;//��λHUB		 
		//					case 0x51:  cmd_device_info();		  break;//��ȡ�豸��Ϣ

							default:	    break;		   	
							}//end switch
						}
						else
						if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX] ==PC_ADDR))   
						{
							switch(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX])
							{    		
		          case 0x10:  cmd_Hub_Rst();		    	break;//��λHUB		 
		//					case 0x51:  cmd_device_info();		  break;//��ȡ�豸��Ϣ

							default:	    break;		   	
							}//end switch
						}
					}
					else//У��Բ���
					{
						UART1_RX_State |= 0xe0;
					}
				}						
				else
				{
					if( (time_sys -time_uart1)>100 )
					{
					UART1_RX_State |= 0xe0;						
//					BUS_Error_Byte(0x11);
					}
					else
					{
					UART1_RX_State = 0x00;
					}
				}
			
		}
		else
		{
				UART1_RX_State |= 0xe0;
		}
		
		if((UART1_RX_State &0xe0)== 0xe0)	//���ճ���
			{
				UART1_RXBUFFE_HEAD +=1;
				UART1_RXBUFFE_HEAD &= UART1_RX_MAX;//����ֽ�
			}
			else
		if(UART1_RX_State ==0)   //�ȴ��������
			{
// 				UART1_RXBUFFE_HEAD +=0;
// 				UART1_RXBUFFE_HEAD &= UART1_RX_MAX;//����ֽ�
			}
			else//�������
			{
			UART1_RXBUFFE_HEAD += (UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX]<<1);
		  UART1_RXBUFFE_HEAD &= UART1_RX_MAX;//����ֽ�
			}
 		UART1_RX_State =0;
		}   //len >0
	
} 
void uart3_cmd (void)
{
	UART3_Receive_Length = UART3_RXBUFFE_LAST - UART3_RXBUFFE_HEAD;
	UART3_Receive_Length &= UART3_RX_MAX;//����ֽ�
/********������ַ���Ϊһ֡���ݵĽ�����ʶ************/
		if(UART3_Receive_Length > 0 )	//ֻ�н��յ�1���������ϲ����ж�
		{
			if(UART3_RXBUFFER[UART3_RXBUFFE_HEAD] == frame_headerC) 	//֡��ʼ��־   
			{
				UART3_RX_State = 1;
				
				if((UART3_Receive_Length >= 2)&&((UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+1)&UART3_RX_MAX]) <= (UART3_Receive_Length>>1))) 	//���ȸպñ�־   
				{
				UART3_RX_State |= 2;
				if((UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+2)&UART3_RX_MAX] ==device.addr) || (UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+2)&UART3_RX_MAX] ==GLOBLE_ADDR)) 
				{			
					switch(UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+4)&UART3_RX_MAX])
					{    		
//          case 0x10:  cmd_Hub_Ctrl();		break;//��λ		 
//					case 0x51:  charge_device_info();		    break;//��ȡ�豸��Ϣ
//		  		case 0x16:  Test_device();		    			break;//
					case 0x16:  cmd3_ShakeHands();		    break;//����

          case 0xE1:  cmd3_Get_State();   			break;//��HUB�ŵ�
          case 0xE2:  cmd3_Set_State();   			break;//  ����HUB�ŵ�FLASH
//          case 0xE3:  cmd_Erase_Flash();      break;//
//          case 0xE4:  cmd_Read_Flash();       break;//
//          case 0xE5:  cmd_Write_Flash();      break;//

					default:	    break;		   	
					}//end switch
				}
				else
				if((UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+4)&UART3_RX_MAX] ==0xe2)) 
				{
//          case 0xE2:  
						cmd3_Set_State();   		//  ����HUB�ŵ�FLASH
				}
			UART3_RXBUFFE_HEAD += (UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+1)&UART3_RX_MAX]<<1);
		  UART3_RXBUFFE_HEAD &= UART3_RX_MAX;//����ֽ�
			}
		}
		else
		{
				UART3_RX_State = 0;
				UART3_RXBUFFE_HEAD +=1;
				UART3_RXBUFFE_HEAD &= UART3_RX_MAX;//����ֽ�
		}
		
		}
	
} 

/* ------------------------------------------end of file---------------------------------------- */
