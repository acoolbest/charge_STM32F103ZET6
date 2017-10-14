//*******************************************
//dwgl for stm32f1XX
//V1.1 20160401
//*******************************************
#include "dwgl_function.h"

//*******************************************
//u16 ×ª×Ö·û
//V1.1 20160401
//*******************************************
void u16toStr(u16 u_data,u8 *p)	 //u16 ×ª×Ö·û
{
	u16 temp,j;
	u8 i;
	temp = u_data;
	if(temp==0)
	{
		*p = '0';
		p++;
	}
	else
	{
		j=10000;
		for(i=0;i<5;i++)
		{
			if(temp/j)
			{break;}
			j =j/10;
		}
		for(;i<5;i++)
		{
			*p=(temp/j)+0X30;
			p++;
			temp = temp%j;	
			j =j/10;
		}
	}
	*p = '\0';
}
//*******************************************
//u16 ×ª×Ö·û ¿ÉÒÔ´øµ¥Î»µÄ
//V1.1 20160401
//*******************************************
void u16toStr_unit(u16 u_data,u8 *p,u8 unit)
{
	u16 temp,j;
	u8 i;
	temp = u_data;
	if(temp==0)
	{
		*p = '0';
		p++;
	}
	else
	{
		j=10000;
		for(i=0;i<5;i++)
		{
			if(temp/j)
			{break;}
			j =j/10;
		}
		for(;i<5;i++)
		{
			*p=(temp/j)+0X30;
			p++;
			temp = temp%j;	
			j =j/10;
		}
	}
	*p = unit;
	p++;
	*p = '\0';
}

//*******************************************
//u16 ×ª×Ö·û ¿ÉÒÔ´øµ¥Î»µÄ
//V1.1 20160401
//*******************************************
void buffer_display(u16 *p,u8 gain,uint16_t color)
{
	// 	u16 i,temp;
	// 	LCD_Clear(0, 50, 320, 130, BACKGROUND);
	// 	if(gain>8){ return;}
	// 	for(i=0;i<320;i++)
	// 	{
	// 	temp = *p;
	// 	temp >>=(gain);
	// 	temp = wave_area_YD-1 - temp;	
	// 	p++;
	// 	point_display(i,temp,color);	
	// 	}

}

//*******************************************
//u16 ×ª×Ö·û ¿ÉÒÔ´øµ¥Î»µÄ
//V1.1 20160401
//*******************************************
void continuous_display(u16 *p,u8 gain,uint16_t color)
{
	// 	u16 i,j,num,temp1,temp2;
	// 	LCD_Clear(0, 50, 320, 130, BACKGROUND);
	// 	if(gain>8){ return;}
	// 	for(i=0;i<320;i++)
	// 	{
	// 	temp1 = *(p+i);
	// 	temp1 >>=(gain);
	// 	temp1 = wave_area_YD-1 - temp1;	
	// 	*(p+i) = temp1;	
	// 	}
	// 	for(i=0;i<319;i++)
	// 	{
	// 	temp1 = *p;
	// 	temp2 = *(p+1);
	// 	point_display(i,temp1,color);	
	// 	if(temp1>temp2)
	// 	{
	// 		num = temp1- temp2;
	// 	for(j=1;j<num;j++)
	// 		{
	// 				point_display(i,temp1-j,color);	
	// 		}
	// 	}
	// 	else
	// 	{
	// 		num = temp2- temp1;
	// 	for(j=1;j<num;j++)
	// 		{
	// 				point_display(i,temp1+j,color);	
	// 		}
	// 	}
	// 	p++;
	// 	}

}
//*******************************************
//u16 ×ª×Ö·û ¿ÉÒÔ´øµ¥Î»µÄ
//V1.1 20160401
//*******************************************

void point_display(u16 x,u16 y,uint16_t color)
{
	// 	LCD_SetCursor(x, y);
	// 	LCD_ILI9341_CMD(0x2c);	         /* Ð´Êý¾Ý */
	// 	LCD_WR_Data(color);
}


//*******************************************
//Ö÷²Ëµ¥
//V1.1 20160401
//*******************************************
void TASK0_display(void)
{
	// 	u16 zb_x,zb_y;
	// 	if((TASK_state&0x80) == 0x80)  //ÓÐ¸üÐÂÈÎÎñ×´Ì¬
	// 	{
	// 	TASK_state &=0x7f; 
	//   LCD_Clear(0, 0, 320, 240, BACKGROUND);
	//   LCD_Clear(0, 0, 39, 32, GREY);
	//   LCD_DispStr(7, 10, (uint8_t *)"MENU", RED);
	//   LCD_Clear(40, 0, 39, 32, GREY);
	//   LCD_DispStr(47, 10, (uint8_t *)"CLR", RED);
	//   LCD_Clear(80, 0, 39, 32, GREY);
	//   LCD_DispStr(87, 10, (uint8_t *)"NEXT", RED);
	// 	
	//   LCD_DispStr(240, 1, (uint8_t *)"TIME:", RED);
	// 	u16toStr_unit( time_s,str_buffer,'S');
	//   LCD_Clear(240+6*6, 1, 6*6, 12, BACKGROUND);
	// 	LCD_DispStr(240+6*6, 1, str_buffer, RED);
	//   LCD_DispStr(240, 14, (uint8_t *)"STEP:", RED);
	// 	u16toStr_unit( step,str_buffer,'T');
	//   LCD_Clear(240+6*6, 14, 6*6, 12, BACKGROUND);
	// 	LCD_DispStr(240+6*6, 14, str_buffer, RED);
	// 		
	//   LCD_Clear(0, 212, 320, 1, GREY);
	//   LCD_Clear(0, 240, 320, 1, GREY);
	//   LCD_Clear(0, 212, 1, 28, GREY);
	//   LCD_Clear(320, 212, 1, 28, GREY);
	//   LCD_DispStr(7, 214, (uint8_t *)"COM:", RED);
	// 	
	//   /* »­4ÌõÖ±Ïß */
	//   LCD_Clear(10, 50, 64, 48, BLUE);
	//   LCD_DispStr(16, 80, (uint8_t *)"TASK1", RED);
	//   LCD_Clear(10+75, 50, 64, 48, BLUE);
	//   LCD_DispStr(16+75, 80, (uint8_t *)"TASK2", RED);
	// 	
	//   LCD_Clear(10+75*2, 50, 64, 48, BLUE);
	//   LCD_DispStr(16+75*2, 80, (uint8_t *)"TASK3", RED);
	//   LCD_Clear(10+75*3, 50, 64, 48, BLUE);
	//   LCD_DispStr(16+75*3, 80, (uint8_t *)"TASK4", RED);
	// 		
	//   LCD_Clear(10, 130, 64, 48, BLUE);
	//   LCD_DispStr(16, 160, (uint8_t *)"TASK5", RED);
	//   LCD_Clear(10+75, 130, 64, 48, BLUE);
	//   LCD_DispStr(16+75, 160, (uint8_t *)"TASK6", RED);
	// 	
	//   LCD_Clear(10+75*2, 130, 64, 48, BLUE);
	//   LCD_DispStr(16+75*2, 160, (uint8_t *)"TASK7", RED);
	//   LCD_Clear(10+75*3, 130, 64, 48, BLUE);
	//   LCD_DispStr(16+75*3, 160, (uint8_t *)"TASK8", RED);
	// }
	// 	
	//   if(touch_flag == 1)			/*Èç¹û´¥±Ê°´ÏÂÁË*/
	// 		{
	//       if(Get_touch_point(&display, Read_2046_2(), &touch_para ) !=DISABLE)      
	//       {	
	// 			/* ÔÚ»­°åÄÚÈ¡É« */
	// 			zb_x = display.x;	
	// 			zb_y = display.y;
	// 				
	// 				if((0<zb_x)&&(zb_x<40)&& (0<zb_y)&&(zb_y<33))			//Èç¹û°´µÄ²Ëµ¥
	// 				{
	// 					Delay_ms(200);		//ÓÃÒ»¸öÑÓÊ±·ÀÉÁÆÁ
	// 					TASK_state |= 0x80;	  //¿ªÊ¼ÈÎÎñ³õÊ¼»¯				
	// 				}
	// 				else
	// 				if((40<zb_x)&&(zb_x<80)&& (0<zb_y)&&(zb_y<33))		//Èç¹û°´µÄÊÇClr
	// 				{
	// 					time_s = 0;
	// 					u16toStr_unit( time_s,str_buffer,'S');
	// 					LCD_Clear(240+6*6, 1, 6*6, 12, BACKGROUND);
	// 					LCD_DispStr(240+6*6, 1, str_buffer, RED);
	// 					
	// 					step = 0;
	// 					u16toStr_unit( step,str_buffer,'T');
	// 					LCD_Clear(240+6*6, 14, 6*6, 12, BACKGROUND);
	// 					LCD_DispStr(240+6*6, 14, str_buffer, RED);
	// 				}
	// 				else
	// 				if((80<zb_x)&&(zb_x<120)&& (0<zb_y)&&(zb_y<33))		//Èç¹û°´µÄÊÇNEXT
	// 				{
	// 					Delay_ms(200);		//ÓÃÒ»¸öÑÓÊ±·ÀÉÁÆÁ
	// 					step++;
	// 					u16toStr_unit( step,str_buffer,'T');
	// 					LCD_DispStr(240+6*6, 14, str_buffer, RED);
	// 				}
	// 				else
	// 				if((10<zb_x)&&(zb_x<74)&& (50<zb_y)&&(zb_y<98))		//Èç¹û°´µÄÊÇTASK1
	// 				{
	// 					TASK_state =0x81; 
	// 					Delay_ms(200);		//ÓÃÒ»¸öÑÓÊ±·ÀÉÁÆÁ
	// 				}
	// 				else
	// 				if((85<zb_x)&&(zb_x<149)&& (50<zb_y)&&(zb_y<98))		//Èç¹û°´µÄÊÇTASK2
	// 				{
	// 					TASK_state =0x82; 
	// 					Delay_ms(200);		//ÓÃÒ»¸öÑÓÊ±·ÀÉÁÆÁ
	// 				}
	// 				else
	// 				if((235<zb_x)&&(zb_x<299)&& (130<zb_y)&&(zb_y<178))		//Èç¹û°´µÄÊÇTAsk8
	// 				{
	// 				  while(Touch_Calibrate() !=0);
	// 					TASK_state |= 0x80;	  //¿ªÊ¼ÈÎÎñ³õÊ¼»¯				
	// 				}
	// 			}//end ÓÐ×ø±êÊ¾
	// 				
	// 		}//end ´¥ÆÁ

}

//*****************************************************
//hub_addr_set Î»Ô½Ð¡ÓÅÏÈ¼¶Ô½¸ß
//*****************************************************
//charge_addr_setµ±ÐÅºÅÏß=0£¬Ê±ÈÏÎªÊÇµØÖ·ÇëÇó£»=1Ê±±íÊ¾ÒÑ¾­È·¶¨µØÖ·¡£
void Addr_Set(void)
{

}
//---------------------------------------------------------------------------------
void  Choose_uart2(u8 port)
{

}
//---------------------------------------------------------------------------------
void Device_Rst(void)
{
	NVIC_SystemReset();
}
void AllPort_PowerInfo(void)
{

}

//---------------------------------------------------------------------------------
void Device_Info(void)
{
	// 	u8 i;
	// 	if(GPIO_ReadInputDataBit(RJ45_IO1_PORT,RJ45_IO1_PIN)==1)
	// 	{
	// 		i = 1;
	// 		UART1_TXBUFFER[0] =  frame_headerD;
	// 		UART1_TXBUFFER[1] =  9;  //len
	// 		UART1_TXBUFFER[2] =  PC_ADDR;
	// 		UART1_TXBUFFER[3] =  hub_addr+i;
	// 		UART1_TXBUFFER[4] =  cmd_set_addr;
	// 		UART1_TXBUFFER[5] =  port_num;
	// 		UART1_TXBUFFER[6] =  device.hub_id[0];
	// 		UART1_TXBUFFER[7] =  device.hub_id[1];
	// 		UART1_TXBUFFER[8] =  device.hub_id[2];
	// 		UART1_TXBUFFER[9] =  device.hub_id[3];
	// 		UART1_TXBUFFER[10] =  device.hub_id[4];
	// 		UART1_TXBUFFER[11] =  device.hub_id[5];
	// 		UART1_TXBUFFER[12] =  device.hub_id[6];
	// 		UART1_TXBUFFER[13] =  device.hub_id[7];
	// 		UART1_TXBUFFER[14] =  i*2 -2;
	// 		UART1_TXBUFFER[15] =  i*2 -1;
	// 		UART1_TXBUFFER[16] = 0;
	// 		for(i=1;i<((UART1_TXBUFFER[1]<<1)-3);i++)
	// 		{
	// 			UART1_TXBUFFER[16] += UART1_TXBUFFER[i];
	// 		}
	// //		UART2_TXBUFFER[16] =  check;
	// 		UART1_TXBUFFER[17] =  frame_last;
	// 		
	// 		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
	// 	}

}
//---------------------------------------------------------------------------------
void cmd_Device_Info(void)
{
	u8 i;
	i= (device.addr>>4);
	i = i*2+4;
	while(time_sys-time_uart1>=i);
	Device_Info();	
}


//---------------------------------------------------------------------------------
void cmd_Erase_Flash(void)
{

}

//---------------------------------------------------------------------------------
void cmd_Read_Flash(void)
{
	u32 addr,i;
	u32 end_addr;
	//	u8 flash_cs;

	//	flash_cs = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX];
	addr =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX];
	addr <<= 8;
	addr +=  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX];
	addr <<= 8;

	end_addr =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
	end_addr <<= 8;
	end_addr +=  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
	end_addr <<= 8;

	for(i=addr;i<end_addr;)  //ÉèÖÃ¿ªÊ¼Ò³£¬Óë½áÊøÒ³
	{
		//		FLASH2_GPIOSPI_Read(i, str_buffer,1024,flash_cs);
		FLASH2_GPIOSPI_Read(i, str_buffer,1024);
		i+=1024;
		UART1_Send_Data(str_buffer,1024);
	}

}

//---------------------------------------------------------------------------------
void cmd_Write_Flash(void)
{
	u32 addr;
	u16 len,i;
	//	u8 flash_cs;

	//	flash_cs = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX];
	addr =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX];
	addr <<= 8;
	addr +=  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX];
	addr <<= 8; //ÉèÖÃ¿ªÊ¼Ò³

	if(addr>=Addr_info)
	{
		FLASH2_GPIOSPI_SER(addr);

		len = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX];
		len <<= 1;
		len -= 10; //³ýÁ½Í·Î²Ð§×Ö½Ú
		for(i=0;i<(len);i++)
		{
			str_buffer[i] = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8+i)&UART1_RX_MAX];
		}
		FLASH2_GPIOSPI_Write(addr, str_buffer, len);

		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  4;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
		UART1_TXBUFFER[5] =  0xff;

		UART1_TXBUFFER[6] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[7] =  frame_last;
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
	}
}

//---------------------------------------------------------------------------------
void cmd_Power_on(void)
{
	u8 i,data;
	data = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX];		
	if(data==device.port_id[0])
	{
		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  6;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
		UART1_TXBUFFER[5] =  device.port_id[0];
		UART1_TXBUFFER[6] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX];
		UART1_TXBUFFER[7] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX];
		UART1_TXBUFFER[8] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
		UART1_TXBUFFER[9] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];

		if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX]!=0)
		{
			Uport_PowerSetTime[0]	= UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
			Uport_PowerSetTime[0]	<<= 8;
			Uport_PowerSetTime[0]	+= UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
			Uport_PowerUseTime[0]	= Uport_PowerSetTime[0];
			Uport_PowerShowTime[0] = 0;
		}

		if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==2) //¿ì³ä ÉÏµç·½Ê½Ñ¡Ôñ2
		{
			Uport_PowerShowTime[0] = Uport_PowerUseTime[0];
			Dport_ChargeOFF(0);
			Dport_ChargeOFF(1);
			Dport_ChargeOFF(2);
			Delay_ms(200);	
			GPIO_ResetBits(EN_KC0_PORT, EN_KC0_PIN); //¿ì³ä 
			//			Delay_ms(100);	
			ADC_BUFFER[20] = 0;  
			checking_port[LCD1_INDEX] &= 0x0f; 
		}
		else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==1) //USB ÉÏµç·½Ê½Ñ¡Ôñ1
		{ 
			Dport_ChargeOFF(0);
			Dport_ChargeOFF(1);
			Dport_ChargeOFF(2);
			Delay_ms(200);	
			GPIO_SetBits(EN_KC0_PORT, EN_KC0_PIN); //USBÉÏµç·½Ê½
			//			Delay_ms(100);	
			ADC_BUFFER[20] = 0;
			checking_port[LCD1_INDEX] &= 0x0f; 
		}

		UART1_TXBUFFER[10] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[10] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[11] =  frame_last;

		i= (device.addr>>4);
		i = i*2+4;
		while((time_sys-time_uart1) <i);
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
	}
	else if(data==device.port_id[1])
	{
		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  6;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
		UART1_TXBUFFER[5] =  device.port_id[1];
		UART1_TXBUFFER[6] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX];
		UART1_TXBUFFER[7] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX];
		UART1_TXBUFFER[8] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
		UART1_TXBUFFER[9] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];

		if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX]!=0)
		{
			Uport_PowerSetTime[1]	= UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
			Uport_PowerSetTime[1]	<<= 8;
			Uport_PowerSetTime[1]	+= UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
			Uport_PowerUseTime[1]	= Uport_PowerSetTime[1];
			Uport_PowerShowTime[1] = 0;
		}
		if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==2) //ÉÏµç·½Ê½Ñ¡Ôñ
		{
			Uport_PowerShowTime[1] = Uport_PowerUseTime[1];
			Dport_ChargeOFF(3);
			Dport_ChargeOFF(4);
			Dport_ChargeOFF(5);
			Delay_ms(200);	
			GPIO_ResetBits(EN_KC1_PORT, EN_KC1_PIN); //¿ì³ä 
			//			Delay_ms(100);	
			ADC_BUFFER[23] = 0;
			checking_port[LCD2_INDEX] &= 0x0f; 
		}
		else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==1) //ÉÏµç·½Ê½Ñ¡Ôñ1
		{
			Dport_ChargeOFF(3);
			Dport_ChargeOFF(4);
			Dport_ChargeOFF(5);
			Delay_ms(200);	
			GPIO_SetBits(EN_KC1_PORT, EN_KC1_PIN); //USBÉÏµç·½Ê½
			//			Delay_ms(100);	
			ADC_BUFFER[23] = 0;
			checking_port[LCD2_INDEX] &= 0x0f; 
		}
		UART1_TXBUFFER[10] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[10] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[11] =  frame_last;

		// 		i= (device.addr>>4);
		// 		i = i*2+4;
		// 		while((time_sys-time_uart1) <i);
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
	}
}

//---------------------------------------------------------------------------------
void cmd_Power_off(void)
{
	u8 i,data;
	data = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX];		
	if(data==device.port_id[0])
	{
		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  4;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
		UART1_TXBUFFER[5] =  device.port_id[0];

		Uport_PowerSetTime[0]	= 0;
		Uport_PowerUseTime[0]	= 0;
		Uport_PowerShowTime[0] = 0;
		UART1_TXBUFFER[6] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[7] =  frame_last;

		i= (device.addr>>4);
		i = i*2+4;
		while((time_sys-time_uart1) <i);
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
	}
	else if(data==device.port_id[1])
	{
		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  4;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
		UART1_TXBUFFER[5] =  device.port_id[1];

		Uport_PowerSetTime[1]	= 0;
		Uport_PowerUseTime[1]	= 0;
		Uport_PowerShowTime[1] = 0;
		UART1_TXBUFFER[6] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[7] =  frame_last;

		// 		i= (device.addr>>4);
		// 		i = i*2+4;
		// 		while((time_sys-time_uart1) <i);
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
	}
}

//------0x11---------------------------------------------------------------------------	
void cmd_File_Requst(void)
{
	// 		str_buffer[0]=     // u8  0X67
	// 		str_buffer[?]=     // u8  ÎÄ¼þ¸öÊý
	// 		str_buffer[?]=     //u16  ÏÂ´ÎÊ¼ÓÃÆðÊ¼Ò³
	// 		str_buffer[?]=     //u8  ÎÄ¼þÇø
	// 		str_buffer[?]=		 //u8  ÎÄ¼þºÅ
	// 		str_buffer[?]=		 //u32 ÎÄ¼þ´óÐ¡
	// 		str_buffer[?]=		 //u64 ÎÄ¼þÃû
	// 		str_buffer[?]=		 //u32 ÎÄ¼þÆðµØÖ·
	// 		str_buffer[?]=     //check
	// 		str_buffer[?]=     //Î²0X99

	u16 i,EN,len;
	u32 f_size,f_temp;
	if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==1)  //¶þÎ¬ÂëÇø
	{
		FLASH2_GPIOSPI_Read (Addr_01min, str_buffer, 256);

		if(str_buffer[0]==frame_headerC)
		{
			len = str_buffer[1];		
			len = len*18+6;
			file_addr = str_buffer[2];//ÏÂ´ÎÊ¼ÓÃÆðÊ¼Ò³ u16
			file_addr <<=8;
			file_addr += str_buffer[3];
			file_addr <<=8;

			f_temp  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+10)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+11)&UART1_RX_MAX];
			f_size = file_addr+f_temp;	

			if((f_size<Addr_01max)&&(str_buffer[1]<=13))
			{
				EN  = 0xff;
				if((f_size&0xff)==0)
				{
					NextFileAddr = f_size;				
				}
				else
				{
					NextFileAddr = (f_size & 0xffffff00);				
					NextFileAddr += 0x100;
				}
				FLASH2_GPIOSPI_SER(Addr_01min);
				for(i=0;i<14;i++)
				{
					str_buffer[len-2+i]=	UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6+i)&UART1_RX_MAX];
				}

				str_buffer[len-2+i]	= (file_addr>>24);
				i++;
				str_buffer[len-2+i]	= (file_addr>>16)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr>>8)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr)&0xff;

				str_buffer[0]	= frame_headerC;
				str_buffer[1]	+= 1;
				str_buffer[2]	= (NextFileAddr>>16)&0xff;
				str_buffer[3]	= (NextFileAddr>>8)&0xff;
				len = str_buffer[1];		
				len = len*18+6;
				str_buffer[len-1]	= 0;
				for(i=1;i<(len-2);i++)
				{
					str_buffer[len-1] += str_buffer[i];
				}
				str_buffer[len-1]	= frame_last;			
				FLASH2_GPIOSPI_Write(Addr_01min, str_buffer, len);	
			}
			else
			{
				EN  = 0x00;
			}

		}
		else
		{
			str_buffer[0]= 0x67;	
			str_buffer[1]= 0x00;	
			//ÎÄ¼þ´ÓÍ··ÅAddr_01min ÎÄ¼þÄÚÈÝ´ÓAddr_01min+0X1000	
			f_temp = Addr_01min + 0x1000;
			str_buffer[2]= ((f_temp>>16)&0xff);	
			str_buffer[3]= ((f_temp>>8)&0xff);	
			len = str_buffer[1];		
			len = len*18+6;
			file_addr = str_buffer[2];
			file_addr <<=8;
			file_addr += str_buffer[3];
			file_addr <<=8;

			f_temp  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+10)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+11)&UART1_RX_MAX];
			f_size = file_addr+f_temp;	

			if((f_size<Addr_01max)&&(str_buffer[1]<=13))
			{
				EN  = 0xff;
				if((f_size&0xff)==0)
				{
					NextFileAddr = f_size;				
				}
				else
				{
					NextFileAddr = (f_size & 0xffffff00);				
					NextFileAddr += 0x100;
				}
				FLASH2_GPIOSPI_SER(Addr_01min);
				for(i=0;i<14;i++)
				{
					str_buffer[len-2+i]=	UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6+i)&UART1_RX_MAX];
				}

				str_buffer[len-2+i]	= (file_addr>>24);
				i++;
				str_buffer[len-2+i]	= (file_addr>>16)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr>>8)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr)&0xff;

				str_buffer[0]	= frame_headerC;
				str_buffer[1]	+= 1;
				str_buffer[2]	= (NextFileAddr>>16)&0xff;
				str_buffer[3]	= (NextFileAddr>>8)&0xff;
				len = str_buffer[1];		
				len = len*18+6;
				str_buffer[len-1]	= 0;
				for(i=1;i<(len-2);i++)
				{
					str_buffer[len-1] += str_buffer[i];
				}
				str_buffer[len-1]	= frame_last;			
				FLASH2_GPIOSPI_Write(Addr_01min, str_buffer, len);	
			}
			else
			{
				EN  = 0x00;
			}
		}
	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==2)//ÎÄ×ÖÇø addr:0x004000-0x00ffffø
	{
		FLASH2_GPIOSPI_Read (Addr_02min, str_buffer, 1024);

		if(str_buffer[0]==frame_headerC)
		{
			len = str_buffer[1];		
			len = len*18+6;
			file_addr = str_buffer[2];//ÏÂ´ÎÊ¼ÓÃÆðÊ¼Ò³ u16
			file_addr <<=8;
			file_addr += str_buffer[3];
			file_addr <<=8;

			f_temp  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+10)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+11)&UART1_RX_MAX];
			f_size = file_addr+f_temp;	

			if((f_size<Addr_02max)&&(str_buffer[1]<=52))
			{
				EN  = 0xff;
				if((f_size&0xff)==0)
				{
					NextFileAddr = f_size;				
				}
				else
				{
					NextFileAddr = (f_size & 0xffffff00);				
					NextFileAddr += 0x100;
				}
				FLASH2_GPIOSPI_SER(Addr_02min);
				for(i=0;i<14;i++)
				{
					str_buffer[len-2+i]=	UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6+i)&UART1_RX_MAX];
				}

				str_buffer[len-2+i]	= (file_addr>>24);
				i++;
				str_buffer[len-2+i]	= (file_addr>>16)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr>>8)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr)&0xff;

				str_buffer[0]	= frame_headerC;
				str_buffer[1]	+= 1;
				str_buffer[2]	= (NextFileAddr>>16)&0xff;
				str_buffer[3]	= (NextFileAddr>>8)&0xff;
				len = str_buffer[1];		
				len = len*18+6;
				str_buffer[len-2]	= 0;
				for(i=1;i<(len-2);i++)
				{
					str_buffer[len-2] += str_buffer[i];
				}
				str_buffer[len-1]	= frame_last;			
				for(i=0;i<1024;)
				{
					FLASH2_GPIOSPI_Write(Addr_02min+i, &str_buffer[i], 256);	
					i += 256;
				}

			}
			else
			{
				EN  = 0x00;
			}

		}
		else
		{
			str_buffer[0]= frame_headerC;	
			str_buffer[1]= 0x00;	
			//ÎÄ¼þ´ÓÍ··ÅAddr_02min ÎÄ¼þÄÚÈÝ´ÓAddr_02min+0X1000	
			f_temp = Addr_02min + 0x1000;
			str_buffer[2]= ((f_temp>>16)&0xff);	
			str_buffer[3]= ((f_temp>>8)&0xff);	
			len = str_buffer[1];		
			len = len*18+6;
			file_addr = str_buffer[2];
			file_addr <<=8;
			file_addr += str_buffer[3];
			file_addr <<=8;

			f_temp  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+10)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+11)&UART1_RX_MAX];
			f_size = file_addr+f_temp;	

			if((f_size<Addr_02max)&&(str_buffer[1]<=52))
			{
				EN  = 0xff;
				if((f_size&0xff)==0)
				{
					NextFileAddr = f_size;				
				}
				else
				{
					NextFileAddr = (f_size & 0xffffff00);				
					NextFileAddr += 0x100;
				}
				FLASH2_GPIOSPI_SER(Addr_02min);
				for(i=0;i<14;i++)
				{
					str_buffer[len-2+i]=	UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6+i)&UART1_RX_MAX];
				}

				str_buffer[len-2+i]	= (file_addr>>24);
				i++;
				str_buffer[len-2+i]	= (file_addr>>16)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr>>8)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr)&0xff;

				str_buffer[0]	= frame_headerC;
				str_buffer[1]	+= 1;
				str_buffer[2]	= (NextFileAddr>>16)&0xff;
				str_buffer[3]	= (NextFileAddr>>8)&0xff;
				len = str_buffer[1];		
				len = len*18+6;
				str_buffer[len-2]	= 0;
				for(i=1;i<(len-2);i++)
				{
					str_buffer[len-2] += str_buffer[i];
				}
				str_buffer[len-1]	= frame_last;			
				// 			for(i=256;i<len;)
				// 				{
				// 				i -= 256;
				// 				FLASH2_GPIOSPI_Write(0x4000, &str_buffer[i], 256);	
				// 				i+= 512;
				// 				}
				// 			if((len&0xff)>0)
				// 				{
				// 				FLASH2_GPIOSPI_Write(0x4000, str_buffer, (len&0xff));
				// 				}		

				for(i=0;i<1024;)
				{
					FLASH2_GPIOSPI_Write(Addr_02min+i, &str_buffer[i], 256);	
					i += 256;
				}
			}
			else
			{
				EN  = 0x00;
			}
		}

	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==3)//²ÊÍ¼Çø
	{
		FLASH2_GPIOSPI_Read (Addr_03min, str_buffer, 1024);

		if(str_buffer[0]==frame_headerC)
		{
			len = str_buffer[1];		
			len = len*18+6;
			file_addr = str_buffer[2];//ÏÂ´ÎÊ¼ÓÃÆðÊ¼Ò³ u16
			file_addr <<=8;
			file_addr += str_buffer[3];
			file_addr <<=8;

			f_temp  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+10)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+11)&UART1_RX_MAX];
			f_size = file_addr+f_temp;	

			if((f_size<Addr_03max)&&(str_buffer[1]<=52))
			{
				EN  = 0xff;
				if((f_size&0xff)==0)
				{
					NextFileAddr = f_size;				
				}
				else
				{
					NextFileAddr = (f_size & 0xffffff00);				
					NextFileAddr += 0x100;
				}
				FLASH2_GPIOSPI_SER(Addr_03min);
				for(i=0;i<14;i++)
				{
					str_buffer[len-2+i]=	UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6+i)&UART1_RX_MAX];
				}

				str_buffer[len-2+i]	= (file_addr>>24);
				i++;
				str_buffer[len-2+i]	= (file_addr>>16)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr>>8)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr)&0xff;

				str_buffer[0]	= frame_headerC;
				str_buffer[1]	+= 1;
				str_buffer[2]	= (NextFileAddr>>16)&0xff;
				str_buffer[3]	= (NextFileAddr>>8)&0xff;
				len = str_buffer[1];		
				len = len*18+6;
				str_buffer[len-2]	= 0;
				for(i=1;i<(len-2);i++)
				{
					str_buffer[len-2] += str_buffer[i];
				}
				str_buffer[len-1]	= frame_last;			

				for(i=0;i<1024;)
				{
					FLASH2_GPIOSPI_Write(Addr_03min+i, &str_buffer[i], 256);	
					i += 256;
				}
			}
			else
			{
				EN  = 0x00;
			}

		}
		else
		{
			str_buffer[0]= frame_headerC;	
			str_buffer[1]= 0x00;	
			//ÎÄ¼þ´ÓÍ··ÅAddr_03min ÎÄ¼þÄÚÈÝ´ÓAddr_03min+0X1000	
			f_temp = Addr_03min + 0x1000;
			str_buffer[2]= ((f_temp>>16)&0xff);	
			str_buffer[3]= ((f_temp>>8)&0xff);	
			len = str_buffer[1];		
			len = len*18+6;
			file_addr = str_buffer[2];
			file_addr <<=8;
			file_addr += str_buffer[3];
			file_addr <<=8;

			f_temp  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+10)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+11)&UART1_RX_MAX];
			f_size = file_addr+f_temp;	

			if((f_size<Addr_03max)&&(str_buffer[1]<=52))
			{
				EN  = 0xff;
				if((f_size&0xff)==0)
				{
					NextFileAddr = f_size;				
				}
				else
				{
					NextFileAddr = (f_size & 0xffffff00);				
					NextFileAddr += 0x100;
				}
				FLASH2_GPIOSPI_SER(Addr_03min);
				for(i=0;i<14;i++)
				{
					str_buffer[len-2+i]=	UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6+i)&UART1_RX_MAX];
				}

				str_buffer[len-2+i]	= (file_addr>>24);
				i++;
				str_buffer[len-2+i]	= (file_addr>>16)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr>>8)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr)&0xff;

				str_buffer[0]	= frame_headerC;
				str_buffer[1]	+= 1;
				str_buffer[2]	= (NextFileAddr>>16)&0xff;
				str_buffer[3]	= (NextFileAddr>>8)&0xff;
				len = str_buffer[1];		
				len = len*18+6;
				str_buffer[len-2]	= 0;
				for(i=1;i<(len-2);i++)
				{
					str_buffer[len-2] += str_buffer[i];
				}
				str_buffer[len-1]	= frame_last;			
				// 			for(i=256;i<len;)
				// 				{
				// 				i -= 256;
				// 				FLASH2_GPIOSPI_Write(0x4000, &str_buffer[i], 256);	
				// 				i+= 512;
				// 				}
				// 			if((len&0xff)>0)
				// 				{
				// 				FLASH2_GPIOSPI_Write(0x4000, str_buffer, (len&0xff));
				// 				}		

				for(i=0;i<1024;)
				{
					FLASH2_GPIOSPI_Write(Addr_03min+i, &str_buffer[i], 256);	
					i += 256;
				}

			}
			else
			{
				EN  = 0x00;
			}
		}
	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==4)//Ã½Ìå1Çø
	{
		FLASH2_GPIOSPI_Read (Addr_04min, str_buffer, 1024);

		if(str_buffer[0]==frame_headerC)
		{
			len = str_buffer[1];		
			len = len*18+6;
			file_addr = str_buffer[2];//ÏÂ´ÎÊ¼ÓÃÆðÊ¼Ò³ u16
			file_addr <<=8;
			file_addr += str_buffer[3];
			file_addr <<=8;

			f_temp  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+10)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+11)&UART1_RX_MAX];
			f_size = file_addr+f_temp;	

			if((f_size<Addr_04max)&&(str_buffer[1]<=52))
			{
				EN  = 0xff;
				if((f_size&0xff)==0)
				{
					NextFileAddr = f_size;				
				}
				else
				{
					NextFileAddr = (f_size & 0xffffff00);				
					NextFileAddr += 0x100;
				}
				FLASH2_GPIOSPI_SER(Addr_04min);
				for(i=0;i<14;i++)
				{
					str_buffer[len-2+i]=	UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6+i)&UART1_RX_MAX];
				}

				str_buffer[len-2+i]	= (file_addr>>24);
				i++;
				str_buffer[len-2+i]	= (file_addr>>16)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr>>8)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr)&0xff;

				str_buffer[0]	= frame_headerC;
				str_buffer[1]	+= 1;
				str_buffer[2]	= (NextFileAddr>>16)&0xff;
				str_buffer[3]	= (NextFileAddr>>8)&0xff;
				len = str_buffer[1];		
				len = len*18+6;
				str_buffer[len-2]	= 0;
				for(i=1;i<(len-2);i++)
				{
					str_buffer[len-2] += str_buffer[i];
				}
				str_buffer[len-1]	= frame_last;			

				for(i=0;i<1024;)
				{
					FLASH2_GPIOSPI_Write(Addr_04min+i, &str_buffer[i], 256);	
					i += 256;
				}
			}
			else
			{
				EN  = 0x00;
			}

		}
		else
		{
			str_buffer[0]= frame_headerC;	
			str_buffer[1]= 0x00;	
			//ÎÄ¼þ´ÓÍ··ÅAddr_04min ÎÄ¼þÄÚÈÝ´ÓAddr_04min+0X1000	
			f_temp = Addr_04min + 0x1000;
			str_buffer[2]= ((f_temp>>16)&0xff);	
			str_buffer[3]= ((f_temp>>8)&0xff);	
			len = str_buffer[1];		
			len = len*18+6;
			file_addr = str_buffer[2];
			file_addr <<=8;
			file_addr += str_buffer[3];
			file_addr <<=8;

			f_temp  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+10)&UART1_RX_MAX];
			f_temp <<=	8;
			f_temp  += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+11)&UART1_RX_MAX];
			f_size = file_addr+f_temp;	

			if((f_size<Addr_04max)&&(str_buffer[1]<=52))
			{
				EN  = 0xff;
				if((f_size&0xff)==0)
				{
					NextFileAddr = f_size;				
				}
				else
				{
					NextFileAddr = (f_size & 0xffffff00);				
					NextFileAddr += 0x100;
				}
				FLASH2_GPIOSPI_SER(Addr_04min);
				for(i=0;i<14;i++)
				{
					str_buffer[len-2+i]=	UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6+i)&UART1_RX_MAX];
				}

				str_buffer[len-2+i]	= (file_addr>>24);
				i++;
				str_buffer[len-2+i]	= (file_addr>>16)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr>>8)&0xff;
				i++;
				str_buffer[len-2+i]	= (file_addr)&0xff;

				str_buffer[0]	= frame_headerC;
				str_buffer[1]	+= 1;
				str_buffer[2]	= (NextFileAddr>>16)&0xff;
				str_buffer[3]	= (NextFileAddr>>8)&0xff;
				len = str_buffer[1];		
				len = len*18+6;
				str_buffer[len-2]	= 0;
				for(i=1;i<(len-2);i++)
				{
					str_buffer[len-2] += str_buffer[i];
				}
				str_buffer[len-1]	= frame_last;			
				// 			for(i=256;i<len;)
				// 				{
				// 				i -= 256;
				// 				FLASH2_GPIOSPI_Write(0x4000, &str_buffer[i], 256);	
				// 				i+= 512;
				// 				}
				// 			if((len&0xff)>0)
				// 				{
				// 				FLASH2_GPIOSPI_Write(0x4000, str_buffer, (len&0xff));
				// 				}		

				for(i=0;i<1024;)
				{
					FLASH2_GPIOSPI_Write(Addr_04min+i, &str_buffer[i], 256);	
					i += 256;
				}

			}
			else
			{
				EN  = 0x00;
			}
		}
	}

	UART1_TXBUFFER[0] =  frame_headerD;
	UART1_TXBUFFER[1] =  4;  //len
	UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
	UART1_TXBUFFER[3] =  device.addr;
	UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
	if(EN==0)
	{
		UART1_TXBUFFER[5] =  0;
	}
	else
	{
		UART1_TXBUFFER[5] =  0xff;
		file_wr = 1;	//ÎÄ¼þ²Ù×÷ÔÊÐí
		if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==1)
		{
			file_wr |= 0x10;
		}
		else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==2)
		{
			file_wr |= 0x20;
		}
		else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==3)
		{
			file_wr |= 0x40;
		}
		else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==4)
		{
			file_wr |= 0x80;
		}
		file_hook = 0;
	}
	UART1_TXBUFFER[6] = 0;//check;
	for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
	{
		UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
	}
	UART1_TXBUFFER[7] =  frame_last;
	UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));

}

//-----0x12----------------------------------------------------------------------------
void cmd_File_Tx(void)
{
	u16 i,len,EN;
	if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==file_hook)&& ((file_wr&0xf) ==1))//¹³×ÓÓÐÃ»ÓÐ¶ÔÉÏ
	{
		file_hook++;
		len = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX]*2;
		if(len==266) //ÊÇ²»ÊÇÂúÒ³
		{
			for(i=0;i<256;i++)
			{
				str_buffer[i] = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7+i)&UART1_RX_MAX];
			}
			FLASH2_GPIOSPI_Write(file_addr, str_buffer, 256);
			file_addr +=256;
		}
		else
		{
			for(i=0;i<(len-10);i++)
			{
				str_buffer[i] = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7+i)&UART1_RX_MAX];
			}
			FLASH2_GPIOSPI_Write(file_addr, str_buffer, len-10);
			file_addr +=len-10;
		}
		EN  = 0xFF;
	}
	else
	{
		EN  = 0x00;
	}
	if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX]==0xff))//´«ÊäÍê³É
	{
		if((file_wr&0x10)== 0x10)
		{
			device.use |=0x10;
		}
		file_wr = 0;

	}

	UART1_TXBUFFER[0] =  frame_headerD;
	UART1_TXBUFFER[1] =  5;  //len
	UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
	UART1_TXBUFFER[3] =  device.addr;
	UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
	UART1_TXBUFFER[5] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX];
	UART1_TXBUFFER[6] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX];

	if(EN==0)
	{
		UART1_TXBUFFER[7] =  0;
	}
	else
	{
		UART1_TXBUFFER[7] =  0xff;
	}
	UART1_TXBUFFER[8] = 0;//check;
	for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
	{
		UART1_TXBUFFER[8] += UART1_TXBUFFER[i];
	}
	UART1_TXBUFFER[9] =  frame_last;
	UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));

}

//----0x13-----------------------------------------------------------------------------
void cmd_File_Recall(void)
{
	u16 i,en,x_load,y_load;
	u32 f_addr,f_size;

	en = 0x00;
	if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==1)//¶þÎ¬ÂëÇø
	{
		FLASH2_GPIOSPI_Read (Addr_01min, str_buffer, 256);
		en = 0x00;
		f_addr = 0;
		f_size = 0;
		if(str_buffer[1]>13)
		{	
			str_buffer[1] = 13;	
		}

		for(i=0;i<str_buffer[1];i++)
		{
			if(str_buffer[0]!=frame_headerC)	
			{		break;	}			
			if(str_buffer[4+18*i+1]==UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX])//È·ÈÏÎÄ¼þºÅ
			{
				en = 0xff; 
				f_size = str_buffer[4+18*i+2];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+3];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+4];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+5];	

				f_addr = str_buffer[4+18*i+14];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+15];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+16];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+17];
				break;
			}
		}

		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  4;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];

		UART1_TXBUFFER[5] =  en;

		UART1_TXBUFFER[6] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[7] =  frame_last;
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));

		if(f_size>0)
		{
			if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX]==device.port_id[0])
			{
				display_flash_BMP (80,56,f_addr,1);
			}
			else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX]==device.port_id[1])
			{
				display_flash_BMP (80,56,f_addr,2);
			}
		}
	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==2)//ÎÄ×ÖÇø
	{
		FLASH2_GPIOSPI_Read (Addr_02min, str_buffer, 1024);
		en = 0x00;
		f_addr = 0;
		f_size = 0;
		if(str_buffer[1]>52)
		{	
			str_buffer[1] = 52;	
		}
		//		UART1_Send_Data(str_buffer,str_buffer[1]*18+6);

		for(i=0;i<str_buffer[1];i++)
		{
			if(str_buffer[0]!=frame_headerC)	
			{		break;	}			
			if(str_buffer[4+18*i+1]==UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX])//È·ÈÏÎÄ¼þºÅ
			{
				en = 0xff; 
				f_size = str_buffer[4+18*i+2];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+3];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+4];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+5];	

				f_addr = str_buffer[4+18*i+14];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+15];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+16];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+17];
				break;
			}
		}

		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  4;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];

		UART1_TXBUFFER[5] =  en;

		UART1_TXBUFFER[6] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[7] =  frame_last;
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
		if(f_size>0)
		{
			if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX]==device.port_id[0])
			{
				FLASH2_GPIOSPI_Read (f_addr, LCD_TxtBuffer[LCD1_INDEX], f_size);
				LCD_TxtBuffer[LCD1_INDEX][2048]=0;
				LCD_TxtBuffer[LCD1_INDEX][2049]=0;
			}
			else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX]==device.port_id[1])
			{
				FLASH2_GPIOSPI_Read (f_addr, LCD_TxtBuffer[LCD2_INDEX], f_size);
				LCD_TxtBuffer[LCD2_INDEX][2048]=0;
				LCD_TxtBuffer[LCD2_INDEX][2049]=0;
			}
		}

	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==3)//²ÊÍ¼Çø
	{
		FLASH2_GPIOSPI_Read (Addr_03min, str_buffer, 1024);
		en = 0x00;
		f_addr = 0;
		f_size = 0;
		if(str_buffer[1]>52)
		{	
			str_buffer[1] = 52;	
		}

		for(i=0;i<str_buffer[1];i++)
		{
			if(str_buffer[0]!=frame_headerC)	
			{		break;	}			
			if(str_buffer[4+18*i+1]==UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX])//È·ÈÏÎÄ¼þºÅ
			{
				en = 0xff; 
				f_size = str_buffer[4+18*i+2];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+3];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+4];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+5];	

				f_addr = str_buffer[4+18*i+14];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+15];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+16];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+17];
				break;
			}
		}

		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  4;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];

		UART1_TXBUFFER[5] =  en;

		UART1_TXBUFFER[6] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[7] =  frame_last;
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));

		if(f_size>0)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);

			if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX]==device.port_id[0])
			{
				if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX]>=7)
				{
					x_load = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
					x_load <<= 8;
					x_load += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
					y_load = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+10)&UART1_RX_MAX];
					y_load <<= 8;
					y_load += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+11)&UART1_RX_MAX];
					display_flash_BMP (x_load,y_load,f_addr,1);
				}
				else
				{	
					display_flash_BMP (0,0,f_addr,1);
				}
			}
			else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX]==device.port_id[1])
			{
				if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX]>=7)
				{
					x_load = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
					x_load <<= 8;
					x_load += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
					y_load = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+10)&UART1_RX_MAX];
					y_load <<= 8;
					y_load += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+11)&UART1_RX_MAX];
					display_flash_BMP (x_load,y_load,f_addr,2);
				}
				else
				{	
					display_flash_BMP (0,0,f_addr,2);
				}
			}
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
		}
	}
	else
	{
		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  4;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];

		UART1_TXBUFFER[5] =  en;

		UART1_TXBUFFER[6] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[7] =  frame_last;
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
	}
}

//----0x14-----------------------------------------------------------------------------
void cmd_File_Erase(void)
{
	u8 i;
	u32 temp;

	UART1_TXBUFFER[0] =  frame_headerD;
	UART1_TXBUFFER[1] =  4;  //len
	UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
	UART1_TXBUFFER[3] =  device.addr;
	UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
	UART1_TXBUFFER[5] =  0x01;
	UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-2] = 0;//check;
	for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
	{
		UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-2] += UART1_TXBUFFER[i];
	}
	UART1_TXBUFFER[7] =  frame_last;
	UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));

	temp =0;
	if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX])==1)//¶þÎ¬ÂëÇø
	{
		for(temp=Addr_01min;temp<Addr_01max;)	
		{
			//			FLASH2_GPIOSPI_SER(temp);
			//			temp +=4096;
			FLASH2_GPIOSPI_SE(temp);
			temp +=0x10000;
		}
	}
	else if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX])==2)//ÎÄ×ÖÇø
	{
		for(temp=Addr_02min;temp<Addr_02max;)	
		{
			//			FLASH2_GPIOSPI_SER(temp);
			//			temp +=4096;
			FLASH2_GPIOSPI_SE(temp);
			temp +=0x10000;
		}
	}
	else if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX])==3)//²ÊÍ¼Çø
	{
		for(temp=Addr_03min;temp<Addr_03max;)	
		{
			//			FLASH2_GPIOSPI_SER(temp);
			//			temp +=4096;
			FLASH2_GPIOSPI_SE(temp);
			temp +=0x10000;
		}
	}
	else if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX])==4)//²ÊÍ¼Çø
	{
		for(temp=Addr_04min;temp<Addr_04max;)	
		{
			//			FLASH2_GPIOSPI_SER(temp);
			//			temp +=4096;
			FLASH2_GPIOSPI_SE(temp);
			temp +=0x10000;
		}
	}
	UART1_TXBUFFER[0] =  frame_headerD;
	UART1_TXBUFFER[1] =  4;  //len
	UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
	UART1_TXBUFFER[3] =  device.addr;
	UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
	UART1_TXBUFFER[5] =  0xff;
	UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-2] = 0;//check;
	for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
	{
		UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-2] += UART1_TXBUFFER[i];
	}
	UART1_TXBUFFER[7] =  frame_last;
	UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));

}

//---------------------------------------------------------------------------------
void cmd_Port_Info(void)   //PC»úÒªµÄÉÏµçÇé¿öÐÅÏ¢
{
	u8 i,data;
	data = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX];
	data %= 2;
	if(data==0)
	{
		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  6;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
		UART1_TXBUFFER[5] =  device.port_id[0];
		UART1_TXBUFFER[6] =  (Uport_PowerSetTime[0]>>8);
		UART1_TXBUFFER[7] =  (Uport_PowerSetTime[0]&0xff);
		UART1_TXBUFFER[8] =  (Uport_PowerUseTime[0]>>8);
		UART1_TXBUFFER[9] =  (Uport_PowerUseTime[0]&0xff);
		UART1_TXBUFFER[10] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[10] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[11] =  frame_last;

		i= (device.addr>>4);
		i = i*2+4;
		while((time_sys-time_uart1) <i);
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
	}
	else
	{
		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  6;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
		UART1_TXBUFFER[5] =  device.port_id[1];
		UART1_TXBUFFER[6] =  (Uport_PowerSetTime[1]>>8);
		UART1_TXBUFFER[7] =  (Uport_PowerSetTime[1]&0xff);
		UART1_TXBUFFER[8] =  (Uport_PowerUseTime[1]>>8);
		UART1_TXBUFFER[9] =  (Uport_PowerUseTime[1]&0xff);
		UART1_TXBUFFER[10] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[10] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[11] =  frame_last;

		// 		i= (device.addr>>4);
		// 		i = i*2+4;
		// 		while((time_sys-time_uart1) <i);
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
	}
}

//---------------------------------------------------------------------------------
//ÏÂ·¢:67  0D  XX(????)  F0  55  02(¶Ë¿Ú¸öÊý)  XX(¶Ë¿ÚºÅ)  U64(??????)  XX(¶Ë¿ÚºÅ)  U64(??????)  XX(???)  99
//·µ»Ø:68  0D  F0  XX(???)  55  02(????)  XX(???X1)  U64(??????)  XX(???X2)  U64(??????)  XX(???)  99
// 		str_buffer[0]=     // u8  0X67
// 		str_buffer[?]=     // u8  ÎÄ¼þ¸öÊý
// 		str_buffer[?]=     //u16  ÏÂ´ÎÊ¼ÓÃÆðÊ¼Ò³
// 		str_buffer[?]=     //u8  ÎÄ¼þÇø
// 		str_buffer[?]=		 //u8  ÎÄ¼þºÅ
// 		str_buffer[?]=		 //u32 ÎÄ¼þ´óÐ¡
// 		str_buffer[?]=		 //u64 ÎÄ¼þÃû
// 		str_buffer[?]=		 //u32 ÎÄ¼þÆðµØÖ·
// 		str_buffer[?]=     //check
// 		str_buffer[?]=     //Î²0X99

void cmd_Device_Check(void)//ºË¶ÔÐÅÏ¢
{
	//Í¬Ê±¶ÔÁ½¸ö¶Ë¿ÚµÄ¶þÎ¬Âë½øÐÐºË¶Ô
	u16 i;
	//	u32 f_temp;
	FLASH2_GPIOSPI_Read (Addr_01min, str_buffer, 256);

	UART1_TXBUFFER[0] =  frame_headerD;
	UART1_TXBUFFER[1] =  0x0d;  //len
	UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
	UART1_TXBUFFER[3] =  device.addr;
	UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
	UART1_TXBUFFER[5] =  0x02;

	if((str_buffer[0]==frame_headerC))
	{			
		if(str_buffer[1]>0)
		{
			UART1_TXBUFFER[6] = str_buffer[5];

			for(i=0;i<8;i++)
			{
				UART1_TXBUFFER[7+i] = str_buffer[10+i];
			}
		}
		else
		{
			UART1_TXBUFFER[6] = 0;

			for(i=0;i<8;i++)
			{
				UART1_TXBUFFER[7+i] = 0;
			}
		}

		if(str_buffer[1]>1)
		{
			UART1_TXBUFFER[15] = str_buffer[5+18];

			for(i=0;i<8;i++)
			{
				UART1_TXBUFFER[16+i] = str_buffer[10+18+i];
			}
		}
		else
		{
			UART1_TXBUFFER[15] = 0;

			for(i=0;i<8;i++)
			{
				UART1_TXBUFFER[16+i] = 0;
			}
		}			
	}
	else
	{
		UART1_TXBUFFER[6] = 0;				
		for(i=0;i<8;i++)
		{
			UART1_TXBUFFER[7+i] = 0;
		}

		UART1_TXBUFFER[15] = 0;
		for(i=0;i<8;i++)
		{
			UART1_TXBUFFER[16+i] = 0;
		}
	}
	UART1_TXBUFFER[24] = 0;//check;
	for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
	{
		UART1_TXBUFFER[24] += UART1_TXBUFFER[i];
	}
	UART1_TXBUFFER[25] =  frame_last;

	UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
}
//---------------------------------------------------------------------------------
void cmd_MediaCtrl(void)
{
	u8 i,en;

	en = 0x01;
	FLASH2_GPIOSPI_Read (Addr_info2, str_buffer, 64);
	//			str_buffer[0] = frame_headerC;

	info2STR.head[0] = frame_headerC;
	info2STR.head[1] = sizeof(info2STR); 
	info2STR.head[1] >>=1;
	info2STR.temp[0] = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
	info2STR.temp[1] = device.addr;
	info2STR.temp[2] = 3;//ÏîÄ¿Êý
	str_buffer[2] = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
	str_buffer[3] = device.addr;
	str_buffer[5] = 8;//ÏîÄ¿Êý

	if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX]==1)  //ÏîÄ¿ºÅ
	{
		for(i=0;i<5;i++)
		{
			info2STR.item1[i] = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5+i)&UART1_RX_MAX];
		}
		LCDC.PSwitch = info2STR.item1_data[0];
		LCDC.LCDPTimeSet = info2STR.item1_data[1];	
		LCDC.LCDPTimeSet <<= 8;	
		LCDC.LCDPTimeSet += info2STR.item1_data[2];	
	}
	if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX]==2)  //ÏîÄ¿ºÅ
	{
		for(i=0;i<5;i++)
		{
			info2STR.item2[i] = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5+i)&UART1_RX_MAX];
		}
		LCDC.SPSwitch = info2STR.item2_data[0];
		LCDC.LCDSPTimeSet = info2STR.item2_data[1];	
		LCDC.LCDSPTimeSet <<= 8;	
		LCDC.LCDSPTimeSet += info2STR.item2_data[2];	
	}
	FLASH2_GPIOSPI_SER(Addr_info2);  ////Ã¿´Î²Á²Á4K
	FLASH2_GPIOSPI_Write(Addr_info2, &info2STR.head[0], sizeof(info2STR));
	for(i=0;i<sizeof(info2STR);i++)
	{
		str_buffer[i] =info2STR.head[i];
	}

	en = 0xff;
	FLASH2_GPIOSPI_Read (Addr_info2, &str_buffer[64], 64);			
	for(i=0;i<sizeof(info2STR);i++)
	{
		if(str_buffer[i]!=str_buffer[64+i])
		{
			en = 0;
		}
	}
	UART1_TXBUFFER[0] =  frame_headerD;
	UART1_TXBUFFER[1] =  4;  //len
	UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
	UART1_TXBUFFER[3] =  device.addr;
	UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
	UART1_TXBUFFER[5] =  en;

	UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-2] = 0;//check;
	for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
	{
		UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-2] += UART1_TXBUFFER[i];
	}
	UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-1] =  frame_last;
	UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));

	FLASH2_GPIOSPI_Read (Addr_04min, str_buffer, 64);  //¶ÁÈ¡Í¼Æ¬ÕÅÊý
	LCDC.PNum = str_buffer[1];
	LCDC.LCDPTime[LCD1_INDEX]=0;		//¹ã¸æ¼ÆÊ±
	LCDC.LCDPTime[LCD2_INDEX]=0;		//¹ã¸æ¼ÆÊ±

}
//---------------------------------------------------------------------------------
void cmd_Device_num(void)
{
	u8 i,en;

	en = 0x01;
	FLASH2_GPIOSPI_Read (Addr_info1, str_buffer, 64);
	if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX]>=10)  //³¤¶È¹»²Å¸üÐÂ£¬²»×ã²»¸üÐÂ
	{
		for(i=0;i<20;i++)
		{
			str_buffer[i] = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+i)&UART1_RX_MAX];
		}

		FLASH2_GPIOSPI_SER(Addr_info1);  ////Ã¿´Î²Á²Á4K
		FLASH2_GPIOSPI_Write(Addr_info1, str_buffer, 64);

		en = 0xff;
		FLASH2_GPIOSPI_Read (Addr_info1, &str_buffer[64], 64);			
		for(i=0;i<20;i++)
		{
			if(str_buffer[i]!=str_buffer[64+i])
			{
				en = 0;
			}
		}
	}
	UART1_TXBUFFER[0] =  frame_headerD;
	UART1_TXBUFFER[1] =  4;  //len
	UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
	UART1_TXBUFFER[3] =  device.addr;
	UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
	UART1_TXBUFFER[5] =  en;

	UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-2] = 0;//check;
	for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
	{
		UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-2] += UART1_TXBUFFER[i];
	}
	UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-1] =  frame_last;
	UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));

	for(i=0;i<12;i++)
	{
		device_num[i]= str_buffer[5+i];
	}
	device_num[i++]= ' ';
	device_num[i++]= 0;

	//		tft_DisplayStr(290, 125, device_num,0x0000,0xffff,3);
}
//---------------------------------------------------------------------------------
void cmd_ShakeHands(void)
{
	u16 i;
	UART1_TXBUFFER[0] =  frame_headerD;
	UART1_TXBUFFER[1] =  4;  //len
	UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
	UART1_TXBUFFER[3] =  device.addr;
	UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
	UART1_TXBUFFER[5] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX];

	UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-2] = 0;//check;
	for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
	{
		UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-2] += UART1_TXBUFFER[i];
	}
	UART1_TXBUFFER[(UART1_TXBUFFER[1]<<1)-1] =  frame_last;

	// 		i= (device.addr>>4);
	// 		i = i*2+4;
	// 		while((time_sys-time_uart1) <i);
	UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
	if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX] ==1)
	{
		UART_BUFFER[0] = 'C';
		UART_BUFFER[1] = 0;
		tft_DisplayStr(0, 0, UART_BUFFER,0XFFFF, 0X0000,3);
	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX] ==2)
	{
		UART_BUFFER[0] = 'S';
		UART_BUFFER[1] = 0;
		tft_DisplayStr(0, 0, UART_BUFFER,0XFFFF, 0X0000,3);
	}
}
//---------------------------------------------------------------------------------
void Hub_Rst(u8 addr,u8 port)
{


}
//---------------------------------------------------------------------------------
void cmd_Hub_Rst(void)
{
	u8 i;

	if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX] ==0x01)  //¸ùHUB¸´Î»
	{
		if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX] ==0xff)||((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX]>>4)==(device.addr>>4)))
		{
			GPIO_ResetBits(HUB0_REST_PORT, HUB0_REST_PIN);  
			GPIO_ResetBits(HUB1_REST_PORT, HUB1_REST_PIN);  
			Delay_ms(100);	
			GPIO_SetBits(HUB0_REST_PORT, HUB0_REST_PIN);  			
			GPIO_SetBits(HUB1_REST_PORT, HUB1_REST_PIN);  			
		}
	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX] ==0x02)  //×ÓHUB¸´Î»
	{
		if(((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX]/2)==(device.addr/2)))
		{
			GPIO_ResetBits(HUB0_REST_PORT, HUB0_REST_PIN);  
			GPIO_ResetBits(HUB1_REST_PORT, HUB1_REST_PIN);  
			Delay_ms(100);	
			GPIO_SetBits(HUB0_REST_PORT, HUB0_REST_PIN);  			
			GPIO_SetBits(HUB1_REST_PORT, HUB1_REST_PIN);  			
		}
	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX] ==0x04)  //ËïHUB¸´Î»
	{
		if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX]==device.addr))
		{
			if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX]==device.port_id[0]))
			{
				GPIO_ResetBits(HUB0_REST_PORT, HUB0_REST_PIN);  
				Delay_ms(100);	
				GPIO_SetBits(HUB0_REST_PORT, HUB0_REST_PIN);  			
			}
			else if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX]==device.port_id[1]))
			{
				GPIO_ResetBits(HUB1_REST_PORT, HUB1_REST_PIN);  
				Delay_ms(100);	
				GPIO_SetBits(HUB1_REST_PORT, HUB1_REST_PIN);  			
			}
		}
	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX] ==0x40)  //µ¥Æ¬»ú¸´Î»E
	{
		if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX] ==0xff)||(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX]==device.addr))
		{
			GPIO_SetBits(SSB0_PORT, SSB0_PIN);
			GPIO_SetBits(SSB1_PORT, SSB1_PIN);
			GPIO_SetBits(SSB2_PORT, SSB2_PIN);
			GPIO_SetBits(SSC0_PORT, SSC0_PIN);
			GPIO_SetBits(SSC1_PORT, SSC1_PIN);
			GPIO_SetBits(SSC2_PORT, SSC2_PIN);
			Delay_ms(200);	
			//»ñÈ¡»ùÏß
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

			FLASH2_GPIOSPI_SER(Addr_info);  ////Ã¿´Î²Á²Á4K
			FLASH2_GPIOSPI_Write(Addr_info, str_buffer, (str_buffer[1]<<1));
			NVIC_SystemReset();
		}
	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX] ==0x80)  //µ¥Æ¬»ú¸´Î»
	{
		if((UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX] ==0xff)||(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+2)&UART1_RX_MAX]==device.addr))
		{
			NVIC_SystemReset();
		}
	}

}

//---------------------------------------------------------------------------------
void Hub_ID_Info(void)
{

}

//---------------------------------------------------------------------------------
void cmd3_Get_State(void)
{
	u8 i;
	UART3_TXBUFFER[0] =  frame_headerD;
	UART3_TXBUFFER[1] =  9;  //len
	UART3_TXBUFFER[2] =  UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+3)&UART3_RX_MAX];
	UART3_TXBUFFER[3] =  device.addr;
	UART3_TXBUFFER[4] =  UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+4)&UART3_RX_MAX];
	UART3_TXBUFFER[5] =  device.port_num;
	UART3_TXBUFFER[6] =  device.hub_id[0];
	UART3_TXBUFFER[7] =  device.hub_id[1];
	UART3_TXBUFFER[8] =  device.hub_id[2];
	UART3_TXBUFFER[9] =  device.hub_id[3];
	UART3_TXBUFFER[10] =  device.hub_id[4];
	UART3_TXBUFFER[11] =  device.hub_id[5];
	UART3_TXBUFFER[12] =  device.hub_id[6];
	UART3_TXBUFFER[13] =  device.hub_id[7];
	UART3_TXBUFFER[14] =  device.port_id[0];
	UART3_TXBUFFER[15] =  device.port_id[1];
	UART3_TXBUFFER[16] = 0;
	for(i=1;i<((UART3_TXBUFFER[1]<<1)-2);i++)
	{
		UART3_TXBUFFER[16] += UART3_TXBUFFER[i];
	}
	//		UART3_TXBUFFER[16] =  check;
	UART3_TXBUFFER[17] =  frame_last;

	UART3_Send_Data(UART3_TXBUFFER,(UART3_TXBUFFER[1]<<1));
}

//---------------------------------------------------------------------------------
void cmd3_Set_State(void)
{
	u8 i;

	testcmd3_time = time_s;
	device.head = 0x67;
	device.addr = UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+2)&UART3_RX_MAX];
	device.port_num = UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+5)&UART3_RX_MAX];
	device.hub_id[0]= UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+6)&UART3_RX_MAX];
	device.hub_id[1]= UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+7)&UART3_RX_MAX];
	device.hub_id[2]= UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+8)&UART3_RX_MAX];
	device.hub_id[3]= UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+9)&UART3_RX_MAX];
	device.hub_id[4]= UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+10)&UART3_RX_MAX];
	device.hub_id[5]= UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+11)&UART3_RX_MAX];
	device.hub_id[6]= UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+12)&UART3_RX_MAX];
	device.hub_id[7]= UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+13)&UART3_RX_MAX];
	device.port_id[0] = UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+14)&UART3_RX_MAX];
	device.port_id[1] = UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+15)&UART3_RX_MAX];

	FLASH2_GPIOSPI_Read (0, str_buffer, 128);
	// 		str_buffer[0]=     //Í·0X67
	// 		str_buffer[?]=     //LEN
	// 		str_buffer[?]=     //ÎÄ¼þÇøu8
	// 		str_buffer[?]=		 //ÎÄ¼þºÅu8
	// 		str_buffer[?]=		 //ÎÄ¼þÃûu64
	// 		str_buffer[?]=		//ÎÄ¼þ´óÐ¡u32
	// 		str_buffer[?]=		//ÎÄ¼þÆðµØÖ·u32
	// 		str_buffer[?]=     //check
	// 		str_buffer[?]=     //Î²0X99

	UART3_TXBUFFER[0] =  frame_headerD;
	UART3_TXBUFFER[1] =  4;  //len
	UART3_TXBUFFER[2] =  UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+3)&UART3_RX_MAX];
	UART3_TXBUFFER[3] =  device.addr;
	UART3_TXBUFFER[4] =  UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+4)&UART3_RX_MAX];
	GPIO_SetBits(RJ45_IO1_PORT, RJ45_IO1_PIN);
	if(device.port_id[0]==str_buffer[3])   //µØÖ·Õý³£ÅÐ¶Ï
	{
		UART3_TXBUFFER[5] =  0xff;
		device.use &= (~0x02);
		// 		GPIO_SetBits(RJ45_IO1_PORT, RJ45_IO1_PIN);
	}
	else
	{
		UART3_TXBUFFER[5] =  0x00;
		device.use |= (0x02);	
		//		GPIO_ResetBits(RJ45_IO1_PORT, RJ45_IO1_PIN);
	}
	UART3_TXBUFFER[6] = 0;
	for(i=1;i<((UART3_TXBUFFER[1]<<1)-2);i++)
	{
		UART3_TXBUFFER[6] += UART3_TXBUFFER[i];
	}
	UART3_TXBUFFER[7] =  frame_last;

	UART3_Send_Data(UART3_TXBUFFER,(UART3_TXBUFFER[1]<<1));

	//  		if(device.port_id[0]!=str_buffer[3])   //µØÖ·Õý³£ÅÐ¶Ï
	// 		{
	// 		GPIO_ResetBits(RJ45_IO1_PORT, RJ45_IO1_PIN);
	// 		}


}
//---------------------------------------------------------------------------------
void cmd3_ShakeHands(void)
{
	u8 i;
	testcmd3_time = time_s;
	UART3_TXBUFFER[0] =  frame_headerD;
	UART3_TXBUFFER[1] =  4;  //len
	UART3_TXBUFFER[2] =  UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+3)&UART3_RX_MAX];
	UART3_TXBUFFER[3] =  device.addr;
	UART3_TXBUFFER[4] =  UART3_RXBUFFER[(UART3_RXBUFFE_HEAD+4)&UART3_RX_MAX];
	UART3_TXBUFFER[5] =  0x00;
	UART3_TXBUFFER[6] = 0;//check;
	for(i=1;i<((UART3_TXBUFFER[1]<<1)-2);i++)
	{
		UART3_TXBUFFER[6] += UART3_TXBUFFER[i];
	}
	UART3_TXBUFFER[7] =  frame_last;

	i= (device.addr>>4);
	i = i*2+4;
	while((time_sys-time_uart1) <i);
	UART3_Send_Data(UART3_TXBUFFER,(UART3_TXBUFFER[1]<<1));
}
//---------------------------------------------------------------------------------
void Dport_ChargeState(void)
{
	u8 i;
	u8 flag;

	for(i=0;i<3;i++)
	{
		flag = 0xff;
		if((Dport_State[i]&0x0c)==0x0c)
		{
			flag = i;
			break;
		}	
	}
	Dport_State[0] = 0;
	Dport_State[1] = 0;
	Dport_State[2] = 0;
	if(flag==0xff)	//Ã»ÓÐ¿ÚÊ¹ÓÃ³äµç¡£  
	{
		for(i=0;i<3;i++)
		{
			if(ADC_BUFFER[i*3+2]<(ADC_Base0[i]-ADC_LINE1))
			{
				Dport_State[i] = 0x0c;
				break;
			}
		}
		for(i=0;i<3;i++)
		{
			if(ADC_BUFFER[i*3+2]<(ADC_Base0[i]-ADC_LINE1))
			{
				Dport_State[i] |= 0x08;
			}
		}
	}
	else		//ÓÐ¿ÚÊ¹ÓÃ¶ÏÕâ¸ö¿Ú
	{
		Dport_State[flag] = 0x0c;
		if(ADC_BUFFER[20]<(ADC_Base0[6]+ADC_LINE2))
		{
			Dport_State[flag] &= ~0x04;
		}

		for(i=0;i<3;i++)
		{
			if(ADC_BUFFER[i*3+2]<(ADC_Base0[i]-ADC_LINE1))
			{
				Dport_State[i] |= 0x08;
			}
		}
	}


	for(i=3;i<6;i++)
	{
		flag = 0xff;
		if((Dport_State[i]&0x0c)==0x0c)
		{
			flag = i;
			break;
		}	
	}
	Dport_State[3] = 0;
	Dport_State[4] = 0;
	Dport_State[5] = 0;
	if(flag==0xff)	//Ã»ÓÐ¿ÚÊ¹ÓÃ³äµç¡£  
	{
		for(i=3;i<6;i++)
		{
			if(ADC_BUFFER[i*3+2]<(ADC_Base0[i]-ADC_LINE1))
			{
				Dport_State[i] = 0x0c;
				break;
			}
		}
		for(i=3;i<6;i++)
		{
			if(ADC_BUFFER[i*3+2]<(ADC_Base0[i]-ADC_LINE1))
			{
				Dport_State[i] |= 0x08;
			}
		}
	}
	else		//ÓÐ¿ÚÊ¹ÓÃ¶ÏÕâ¸ö¿Ú
	{
		Dport_State[flag] = 0x0c;
		if(ADC_BUFFER[23]<(ADC_Base0[7]+ADC_LINE2))
		{
			Dport_State[flag] &= ~0x04;
		}
		for(i=3;i<6;i++)
		{
			if(ADC_BUFFER[i*3+2]<(ADC_Base0[i]-ADC_LINE1))
			{
				Dport_State[i] |= 0x08;
			}
		}

	}

}
//---------------------------------------------------------------------------------
//0x00  ¿ªÊ¼¼ì¼ì²â
//0x01  ¼ä¸ô200msÖÓ¼ì²â
//0x02  ¼ä¸ô300msÖÓ¼ìÈ·¶¨¶Ë¿Ú
//0x03  ¼ä¸ô2000msÖÓ¼ìÈ·¶¨¶Ë¿Ú
//0x04  ¼ä¸ô100msÖÓ¼ì²â³äµçÏÂÏÞ
//0x05  ¼ä¸ô100msÖÓ¼ì²âµçÁ÷ÊÇ·ñ±ä´ó¡£È«¿ª
//0x06  ¼ä¸ô100msÖÓ¼ì²âµçÁ÷ÊÇ·ñ±ä´ó¡£¹Ø¶ÏÔ­±¾Ã»ÓÐÓÃµÄ¶Ë¿Ú

u8 u8_sampling_times = 0~3;

ADC1_Pointer[ADC1_channel[ADC1_channel_index]*ADC_SAMPLING_TIMES+u8_sampling_times] = ADC_GetConversionValue(ADC1);

ADC3_Pointer[ADC3_channel[ADC3_channel_index]*ADC_SAMPLING_TIMES+u8_sampling_times] = ADC_GetConversionValue(ADC3);

void Dport_ChargeStateB(u8 lcd_index)
{
	u8 i;
	u8 i_min = lcd_index*3;
	u8 i_max = lcd_index*3+3;
	
	u8 flag;
	
	static u32 time_sys_temp[2];

	if((checking_port[lcd_index]&0xf0)==0x00)  //¿ªÆô¼ì²â
	{
		time_sys_temp[lcd_index] = time_sys;
		checking_port[lcd_index] = 0x10+0x0f;  //°ÑUSBÖÃ¿Õ
		flag = checking_port[lcd_index]&0x0f;

		for(i=i_min;i<i_max;i++) Dport_State[i] = 0x01;

		usb_mutually_exclusive_power_on(LCD1_INDEX);

		for(i=i_min;i<i_max;i++) usb_power_ctrl(i, USB_POWER_ON);
	}
	else if((checking_port[LCD1_INDEX]&0xf0)==0x10)  //¼ì²âÓÐÉè±¸
	{
		if(time_sys-time_sys_temp[lcd_index]>=200)
		{
			time_sys_temp[lcd_index] = time_sys;
			flag = checking_port[LCD1_INDEX]&0x0f;
			if(ADC_BUFFER[20]>(ADC_Base0[6]+ADC_LINE2))
			{
				flag = 0;
				Dport_State[flag] = 0x0c;
				checking_port[LCD1_INDEX] = 0x20+flag;		//¼ì²âµ½¶Ë¿Ú
				Dport_ChargeOFF(flag);
			}
		}
	}
	else if((checking_port[LCD1_INDEX]&0xf0)==0x20)  //¼ì²âÈ·¶¨¶Ë¿Ú
	{
		if(time_sys-time_sys_temp[lcd_index]>=300)
		{
			time_sys_temp[lcd_index] = time_sys;
			flag = checking_port[LCD1_INDEX]&0x0f;
			if(ADC_BUFFER[20]<(ADC_Base0[6]+ADC_LINE2))
			{
				Dport_State[flag] = 0x0c;
				checking_port[LCD1_INDEX] = 0x30+flag;		//¼ì²âµ½¶Ë¿Ú
				usb_mutually_exclusive_power_on(LCD1_INDEX);  //»¥²ðÉÏµç´øUSB
			}
			else
			{
				Dport_State[flag] = 0x00;
				if((flag>=2))  
				{
					checking_port[LCD1_INDEX]=0;		
				}
				else
				{
					checking_port[LCD1_INDEX]=0x20+flag+1;
					Dport_ChargeOFF(flag+1);
				}
			}

		}
	}
	else if((checking_port[LCD1_INDEX]&0xf0)==0x30)  //ÔÙ´ÎÈ·¶¨¶Ë¿Ú
	{
		if(time_sys-time_sys_temp[lcd_index]>=2000)
		{
			time_sys_temp[lcd_index] = time_sys;
			flag = checking_port[LCD1_INDEX]&0x0f;
			if(ADC_BUFFER[20]>(ADC_Base0[6]+ADC_LINE2))
			{
				Dport_State[flag] = 0x0c;
				checking_port[LCD1_INDEX] = 0x40+flag;		//¼ì²âµ½¶Ë¿Ú
			}
			else
			{
				Dport_State[flag] = 0x00;
				checking_port[LCD1_INDEX] = 0;		//¼ì²â²»µ½¶Ë¿Ú
			}
		}
	}
	else if((checking_port[LCD1_INDEX]&0xf0)==0x40) //¼ì²âÕýÔÚ³äµçµÄÉè±¸
	{
		if(time_sys-time_sys_temp[lcd_index]>=100)
		{
			time_sys_temp[lcd_index] = time_sys;
			flag = checking_port[LCD1_INDEX]&0x0f;
			if(ADC_BUFFER[20]<(ADC_Base0[6]+ADC_LINE3))
			{
				LOW_portB++;
				if(LOW_portB>3)
				{
					Dport_State[0] = 0x03;
					Dport_State[1] = 0x03;
					Dport_State[2] = 0x03;
					Dport_State[flag] = 0x0c;
					//					Dport_ChargeOFF(flag);
					checking_port[LCD1_INDEX] = 0x50+flag;		//¼ì²â²»µ½¶Ë¿Ú
					Dport_ChargeON(0);
					Dport_ChargeON(1);
					Dport_ChargeON(2);		
				}
			}
			else
			{
				Dport_State[0] = 0x02;
				Dport_State[1] = 0x02;
				Dport_State[2] = 0x02;
				Dport_State[flag] = 0x0c;
				LOW_portB=0;
			}
		}
	}		
	else if((checking_port[LCD1_INDEX]&0xf0)==0x50) //¼ì²âÓÐÉè±¸
	{
		if(time_sys-time_sys_temp[lcd_index]>=50)
		{
			time_sys_temp[lcd_index] = time_sys;
			flag = checking_port[LCD1_INDEX]&0x0f;
			if(ADC_BUFFER[20]>(ADC_Base0[6]+ADC_LINE2))
			{
				Dport_State[flag] = 0x0c;
				checking_port[LCD1_INDEX] = 0x60+flag;		//¼ì²âµ½¶Ë¿Ú
				for(i=0;i<3;i++)
				{
					if(Dport_State[i]==0x03)
					{
						Dport_ChargeOFF(i);
						Dport_State[i]=0x04;
						break;
					}
				}
			}
		}
	}		
	else if((checking_port[LCD1_INDEX]&0xf0)==0x60) //¼ì²âÈ·¶¨¶Ë¿Ú
	{
		if(time_sys-time_sys_temp[lcd_index]>=50)
		{
			time_sys_temp[lcd_index] = time_sys;
			flag = checking_port[LCD1_INDEX]&0x0f;

			if(ADC_BUFFER[20]<(ADC_Base0[6]+ADC_LINE3))
			{
				for(i=0;i<3;i++)
				{
					if(Dport_State[i]==0x04)//·´À¡¼ì²â½á¹û×´Ì¬
					{
						flag = i;
						Dport_State[0] = 0x06;
						Dport_State[1] = 0x06;
						Dport_State[2] = 0x06;
						Dport_State[flag] = 0x0c;
						//							checking_port[LCD1_INDEX] = 0x30+flag;		//¼ì²âµ½¶Ë¿Ú
						checking_port[LCD1_INDEX] = 0x10+flag;		//¼ì²âµ½¶Ë¿Ú
						usb_mutually_exclusive_power_on(LCD1_INDEX);  //»¥²ðÉÏµç´øUSB
						Dport_ChargeON(0);
						Dport_ChargeON(1);
						Dport_ChargeON(2);
						break;
					}
				}					
			}
			else
			{
				for(i=0;i<3;i++)//·´À¡¼ì²â½á¹û×´Ì¬
				{
					if(Dport_State[i]==0x04)
					{
						Dport_State[i] = 0x05;
					}
				}

				for(i=0;i<3;i++)
				{						
					if(Dport_State[i]==0x03)
					{
						Dport_ChargeOFF(i);
						Dport_State[i]=0x04;
						break;
					}						
				}
				if(i>=3)//Ã»ÓÐ´ý¼ì²âµÄ¶Ë¿Ú
				{
					Dport_State[flag] = 0x0c;
					Dport_ChargeON(flag);
					checking_port[LCD1_INDEX] = 0x40+flag;		//Õý³äµç
				}

			}

		}
	}		

}
//------------------------------------------------------
void Dport_ChargeStateC(void)
{
	u8 i;
	u8 flag;
	if((checking_port[LCD2_INDEX]&0xf0)==0x00)  //¿ªÆô¼ì²â
	{
		time_sys_temp5 = time_sys;
		checking_port[LCD2_INDEX] = 0x10+0x0f;  //°ÑUSBÖÃ¿Õ
		flag = checking_port[LCD2_INDEX]&0x0f;
		Dport_State[3] = 0x01;
		Dport_State[4] = 0x01;
		Dport_State[5] = 0x01;
		usb_mutually_exclusive_power_on(LCD2_INDEX);
		Dport_ChargeON(3);
		Dport_ChargeON(4);
		Dport_ChargeON(5);

	}
	else if((checking_port[LCD2_INDEX]&0xf0)==0x10)  //¼ì²âÓÐÉè±¸
	{
		if(time_sys-time_sys_temp5>=200)
		{
			time_sys_temp5 = time_sys;
			flag = checking_port[LCD2_INDEX]&0x0f;
			if(ADC_BUFFER[23]>(ADC_Base0[7]+ADC_LINE2))
			{
				flag = 3;
				Dport_State[flag] = 0x0c;
				checking_port[LCD2_INDEX] = 0x20+flag;		//¼ì²âµ½¶Ë¿Ú
				Dport_ChargeOFF(flag);
			}
		}
	}
	else if((checking_port[LCD2_INDEX]&0xf0)==0x20)  //¼ì²âÈ·¶¨¶Ë¿Ú
	{
		if(time_sys-time_sys_temp5>=300)
		{
			time_sys_temp5 = time_sys;
			flag = checking_port[LCD2_INDEX]&0x0f;
			if(ADC_BUFFER[23]<(ADC_Base0[7]+ADC_LINE2))
			{
				Dport_State[flag] = 0x0c;
				checking_port[LCD2_INDEX] = 0x30+flag;		//¼ì²âµ½¶Ë¿Ú
				usb_mutually_exclusive_power_on(LCD2_INDEX);  //»¥²ðÉÏµç´øUSB
			}
			else
			{
				Dport_State[flag] = 0x00;
				if((flag>=5))  
				{
					checking_port[LCD2_INDEX]=3;		
				}
				else
				{
					checking_port[LCD2_INDEX]=0x20+flag+1;
					Dport_ChargeOFF(flag+1);
				}
			}

		}
	}
	else if((checking_port[LCD2_INDEX]&0xf0)==0x30)  //ÔÙ´ÎÈ·¶¨¶Ë¿Ú
	{
		if(time_sys-time_sys_temp5>=2000)
		{
			time_sys_temp5 = time_sys;
			flag = checking_port[LCD2_INDEX]&0x0f;
			if(ADC_BUFFER[23]>(ADC_Base0[7]+ADC_LINE2))
			{
				Dport_State[flag] = 0x0c;
				checking_port[LCD2_INDEX] = 0x40+flag;		//¼ì²âµ½¶Ë¿Ú
			}
			else
			{
				Dport_State[flag] = 0x00;
				checking_port[LCD2_INDEX] = 3;		//¼ì²â²»µ½¶Ë¿Ú
			}
		}
	}
	else if((checking_port[LCD2_INDEX]&0xf0)==0x40) //¼ì²âÕýÔÚ³äµçµÄÉè±¸
	{
		if(time_sys-time_sys_temp5>=100)
		{
			time_sys_temp5 = time_sys;
			flag = checking_port[LCD2_INDEX]&0x0f;
			if(ADC_BUFFER[23]<(ADC_Base0[7]+ADC_LINE3))
			{
				LOW_portC++;
				if(LOW_portC>3)
				{
					Dport_State[3] = 0x03;
					Dport_State[4] = 0x03;
					Dport_State[5] = 0x03;						
					Dport_State[flag] = 0x0c;
					checking_port[LCD2_INDEX] = 0x50+flag;		//¼ì²â²»µ½¶Ë¿Ú//¿ªÆðËùÓÐ¿ª¹Ø
					Dport_ChargeON(3);
					Dport_ChargeON(4);
					Dport_ChargeON(5);		
				}
			}
			else
			{
				Dport_State[3] = 0x02;
				Dport_State[4] = 0x02;
				Dport_State[5] = 0x02;
				Dport_State[flag] = 0x0c;
				LOW_portC=0;
			}
		}
	}		
	else if((checking_port[LCD2_INDEX]&0xf0)==0x50) //¼ì²âÓÐÉè±¸
	{
		if(time_sys-time_sys_temp5>=50)
		{
			time_sys_temp5 = time_sys;
			flag = checking_port[LCD2_INDEX]&0x0f;
			if(ADC_BUFFER[23]>(ADC_Base0[7]+ADC_LINE2))
			{
				Dport_State[flag] = 0x0c;
				checking_port[LCD2_INDEX] = 0x60+flag;		//¼ì²âµ½¶Ë¿Ú
				for(i=3;i<6;i++)
				{
					if(Dport_State[i]==0x03)
					{
						Dport_ChargeOFF(i);    //È·¶¨ÊÇ²»ÊÇÔ­¶Ë¿Ú
						Dport_State[i]=0x04;
						break;
					}
				}
			}
		}
	}		
	else if((checking_port[LCD2_INDEX]&0xf0)==0x60) //¼ì²âÈ·¶¨¶Ë¿Ú
	{
		if(time_sys-time_sys_temp5>=50)
		{
			time_sys_temp5 = time_sys;
			flag = checking_port[LCD2_INDEX]&0x0f;

			if(ADC_BUFFER[23]<(ADC_Base0[7]+ADC_LINE3))
			{
				for(i=3;i<6;i++)
				{
					if(Dport_State[i]==0x04)//·´À¡¼ì²â½á¹û×´Ì¬
					{
						flag = i;
						Dport_State[3] = 0x06;
						Dport_State[4] = 0x06;
						Dport_State[5] = 0x06;
						Dport_State[flag] = 0x0c;
						//							checking_port[LCD2_INDEX] = 0x30+flag;		//¼ì²âµ½¶Ë¿Ú
						checking_port[LCD2_INDEX] = 0x10+flag;		//¼ì²âµ½¶Ë¿Ú
						usb_mutually_exclusive_power_on(LCD2_INDEX);  //»¥²ðÉÏµç´øUSB
						Dport_ChargeON(3);
						Dport_ChargeON(4);
						Dport_ChargeON(5);
						break;
					}
				}					
			}
			else
			{
				for(i=3;i<6;i++)//·´À¡¼ì²â½á¹û×´Ì¬
				{
					if(Dport_State[i]==0x04)
					{
						Dport_State[i] = 0x05;
					}
				}

				for(i=3;i<6;i++)
				{						
					if(Dport_State[i]==0x03)
					{
						Dport_ChargeOFF(i);
						Dport_State[i]=0x04;
						break;
					}						
				}
				if(i>=6)//Ã»ÓÐ´ý¼ì²âµÄ¶Ë¿Ú
				{
					Dport_State[flag] = 0x0c;
					Dport_ChargeON(flag);
					checking_port[LCD2_INDEX] = 0x40+flag;		//Õý³äµç
				}

			}

		}
	}		

}
//-----------------------------------------------------
void Get_ADC_BaseLine(void)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		ADC_Base0[i] = ADC_BUFFER[i*3+2];
	}
}
//-----------------------------------------------------

void ChargeCtrl_B(void)
{
	u8 i;
	u8 flag;
	flag = 0xff;
	if(Uport_PowerUseTime[0]>0)
	{
		flag = 0xff;
		for(i=0;i<3;i++)
		{
			if((Dport_State[i]&0x0c)==0x0c)
			{
				flag = i;
			}
		}
	}

	if(flag==0)	//Ã»ÓÐ¿ÚÊ¹ÓÃ³äµç¡£  
	{
		//GPIO_SetBits(SSB0_PORT, SSB0_PIN);
		GPIO_SetBits(SSB1_PORT, SSB1_PIN);
		GPIO_SetBits(SSB2_PORT, SSB2_PIN);

		GPIO_SetBits(USB_DB0_PORT, USB_DB0_PIN);
		//GPIO_ResetBits(USB_DB0_PORT, USB_DB0_PIN);
		GPIO_SetBits(USB_DB1_PORT, USB_DB1_PIN);
		//GPIO_ResetBits(USB_DB1_PORT, USB_DB1_PIN);

		GPIO_ResetBits(SSB0_PORT, SSB0_PIN);
	}
	else if(flag==1)	//Ã»ÓÐ¿ÚÊ¹ÓÃ³äµç¡£  
	{
		GPIO_SetBits(SSB0_PORT, SSB0_PIN);
		//GPIO_SetBits(SSB1_PORT, SSB1_PIN);
		GPIO_SetBits(SSB2_PORT, SSB2_PIN);

		//GPIO_SetBits(USB_DB0_PORT, USB_DB0_PIN);
		GPIO_ResetBits(USB_DB0_PORT, USB_DB0_PIN);
		GPIO_SetBits(USB_DB1_PORT, USB_DB1_PIN);
		//GPIO_ResetBits(USB_DB1_PORT, USB_DB1_PIN);

		GPIO_ResetBits(SSB1_PORT, SSB1_PIN);
	}
	else if(flag==2)	//Ã»ÓÐ¿ÚÊ¹ÓÃ³äµç¡£  
	{
		GPIO_SetBits(SSB0_PORT, SSB0_PIN);
		GPIO_SetBits(SSB1_PORT, SSB1_PIN);
		//GPIO_SetBits(SSB2_PORT, SSB2_PIN);

		GPIO_SetBits(USB_DB0_PORT, USB_DB0_PIN);
		//GPIO_ResetBits(USB_DB0_PORT, USB_DB0_PIN);
		//GPIO_SetBits(USB_DB1_PORT, USB_DB1_PIN);
		GPIO_ResetBits(USB_DB1_PORT, USB_DB1_PIN);

		GPIO_ResetBits(SSB2_PORT, SSB2_PIN);
	}
	else
	{
		//GPIO_SetBits(USB_DB0_PORT, USB_DB0_PIN);
		GPIO_ResetBits(USB_DB0_PORT, USB_DB0_PIN);
		//GPIO_SetBits(USB_DB1_PORT, USB_DB1_PIN);
		GPIO_ResetBits(USB_DB1_PORT, USB_DB1_PIN);

		GPIO_SetBits(SSB0_PORT, SSB0_PIN);
		GPIO_SetBits(SSB1_PORT, SSB1_PIN);
		GPIO_SetBits(SSB2_PORT, SSB2_PIN);
	}	
}
//-----------------------------------------------------
void ChargeCtrl_C(void)
{
	u8 i;
	u8 flag;
	flag = 0xff;
	if(Uport_PowerUseTime[1]>0)
	{
		flag = 0xff;
		for(i=3;i<6;i++)
		{
			if((Dport_State[i]&0x0c)==0x0c)
			{
				flag = i;
			}
		}
	}

	if(flag==3)	//Ã»ÓÐ¿ÚÊ¹ÓÃ³äµç¡£  
	{
		//GPIO_SetBits(SSC0_PORT, SSC0_PIN);
		GPIO_SetBits(SSC1_PORT, SSC1_PIN);
		GPIO_SetBits(SSC2_PORT, SSC2_PIN);

		GPIO_SetBits(USB_DC0_PORT, USB_DC0_PIN);
		//GPIO_ResetBits(USB_DC0_PORT, USB_DC0_PIN);
		GPIO_SetBits(USB_DC1_PORT, USB_DC1_PIN);
		//GPIO_ResetBits(USB_DC1_PORT, USB_DC1_PIN);

		GPIO_ResetBits(SSC0_PORT, SSC0_PIN);
	}
	else if(flag==4)	//Ã»ÓÐ¿ÚÊ¹ÓÃ³äµç¡£  
	{
		GPIO_SetBits(SSC0_PORT, SSC0_PIN);
		//GPIO_SetBits(SSC1_PORT, SSC1_PIN);
		GPIO_SetBits(SSC2_PORT, SSC2_PIN);

		//GPIO_SetBits(USB_DC0_PORT, USB_DC0_PIN);
		GPIO_ResetBits(USB_DC0_PORT, USB_DC0_PIN);
		GPIO_SetBits(USB_DC1_PORT, USB_DC1_PIN);
		//GPIO_ResetBits(USB_DC1_PORT, USB_DC1_PIN);

		GPIO_ResetBits(SSC1_PORT, SSC1_PIN);
	}
	else if(flag==5)	//Ã»ÓÐ¿ÚÊ¹ÓÃ³äµç¡£  
	{
		GPIO_SetBits(SSC0_PORT, SSC0_PIN);
		GPIO_SetBits(SSC1_PORT, SSC1_PIN);
		//GPIO_SetBits(SSC2_PORT, SSC2_PIN);

		GPIO_SetBits(USB_DC0_PORT, USB_DC0_PIN);
		//GPIO_ResetBits(USB_DC0_PORT, USB_DC0_PIN);
		//GPIO_SetBits(USB_DC1_PORT, USB_DC1_PIN);
		GPIO_ResetBits(USB_DC1_PORT, USB_DC1_PIN);

		GPIO_ResetBits(SSC2_PORT, SSC2_PIN);
	}
	else
	{
		//GPIO_SetBits(USB_DC0_PORT, USB_DC0_PIN);
		GPIO_ResetBits(USB_DC0_PORT, USB_DC0_PIN);
		//GPIO_SetBits(USB_DC1_PORT, USB_DC1_PIN);
		GPIO_ResetBits(USB_DC1_PORT, USB_DC1_PIN);

		GPIO_SetBits(SSC0_PORT, SSC0_PIN);
		GPIO_SetBits(SSC1_PORT, SSC1_PIN);
		GPIO_SetBits(SSC2_PORT, SSC2_PIN);
	}

}
void Dport_ChargeON(u8 port)
{
	//	u8 flag;
	if(port==0)	//0¿ÚÊ¹ÓÃ³äµç¡£  
	{
		GPIO_ResetBits(SSB0_PORT, SSB0_PIN);
	}
	else if(port==1)	//1¿ÚÊ¹ÓÃ³äµç¡£  
	{
		GPIO_ResetBits(SSB1_PORT, SSB1_PIN);
	}
	else if(port==2)	//2¿ÚÊ¹ÓÃ³äµç¡£  
	{
		GPIO_ResetBits(SSB2_PORT, SSB2_PIN);
	}
	else if(port==3)	//3¿ÚÊ¹ÓÃ³äµç¡£  
	{
		GPIO_ResetBits(SSC0_PORT, SSC0_PIN);
	}
	else if(port==4)	//4¿ÚÊ¹ÓÃ³äµç¡£  
	{
		GPIO_ResetBits(SSC1_PORT, SSC1_PIN);
	}
	else if(port==5)	//5¿ÚÊ¹ÓÃ³äµç¡£  
	{
		GPIO_ResetBits(SSC2_PORT, SSC2_PIN);
	}
}
void Dport_ChargeOFF(u8 port)
{
	if(port==0)	//0¿ÚÊ¹ÓÃ¶Ïµç¡£  
	{
		GPIO_SetBits(SSB0_PORT, SSB0_PIN);
		// 		GPIO_SetBits(SSB1_PORT, SSB1_PIN);
		// 		GPIO_SetBits(SSB2_PORT, SSB2_PIN);
	}
	else if(port==1)	//1¿ÚÊ¹ÓÃ¶Ïµç¡£    
	{
		// 		GPIO_SetBits(SSB0_PORT, SSB0_PIN);
		GPIO_SetBits(SSB1_PORT, SSB1_PIN);
		//		GPIO_SetBits(SSB2_PORT, SSB2_PIN);
	}
	else if(port==2)	//2¿ÚÊ¹ÓÃ¶Ïµç¡£  
	{
		//		GPIO_SetBits(SSB0_PORT, SSB0_PIN);
		//		GPIO_SetBits(SSB1_PORT, SSB1_PIN);
		GPIO_SetBits(SSB2_PORT, SSB2_PIN);
	}
	else if(port==3)	//3¿ÚÊ¹ÓÃ¶Ïµç¡£   
	{
		GPIO_SetBits(SSC0_PORT, SSC0_PIN);
		// 		GPIO_SetBits(SSC1_PORT, SSC1_PIN);
		// 		GPIO_SetBits(SSC2_PORT, SSC2_PIN);
	}
	else if(port==4)	//4¿ÚÊ¹ÓÃ¶Ïµç¡£   
	{
		//		GPIO_SetBits(SSC0_PORT, SSC0_PIN);
		GPIO_SetBits(SSC1_PORT, SSC1_PIN);
		//		GPIO_SetBits(SSC2_PORT, SSC2_PIN);
	}
	else if(port==5)	//5¿ÚÊ¹ÓÃ¶Ïµç¡£    
	{
		//		GPIO_SetBits(SSC0_PORT, SSC0_PIN);
		//		GPIO_SetBits(SSC1_PORT, SSC1_PIN);
		GPIO_SetBits(SSC2_PORT, SSC2_PIN);
	}
}
void DisplayADC_BL(unsigned int x, unsigned int y, u16 *s,u16 PenColor, u16 BackColor,u8 cs)
{
	u8 i,j;
	u32 temp;
	for(i=0;i<3;i++)
	{
		temp = *s++;
		j=0;
		temp *= 6600;//1000±¶·Å´ó
		temp /=4096;
		UART_BUFFER[j++] = temp%10000/1000+'0';
		UART_BUFFER[j++] = temp%1000/100+'0';
		UART_BUFFER[j++] = temp%100/10+'0';
		UART_BUFFER[j++] = temp%10+'0';
		UART_BUFFER[j++] = 0;
		tft_DisplayStr(x-i*20, 0, UART_BUFFER,POINT_COLOR, BACK_COLOR,cs);
	}
	// 	for(i=0;i<3;i++)
	// 	{
	// 		temp = *s++;
	// 		temp *= 66;//10±¶·Å´ó
	// 		temp /=4096;
	// 		UART_BUFFER[i++] = temp%100/10;
	// 		UART_BUFFER[i++] = '.';
	// 		UART_BUFFER[i++] = temp%10;
	// 		UART_BUFFER[i++] = 0;
	// 		tft_DisplayStr(x-i*20, 0, UART_BUFFER,POINT_COLOR, BACK_COLOR,2);
	// 	}
}

//-------------------------------------
void cmd_Get_ADC(void)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		UART1_TXBUFFER[2*i] = (ADC_BUFFER[3*i+2]>>8);
		UART1_TXBUFFER[2*i+1] = (ADC_BUFFER[3*i+2]&0xff);
	}

	for(i=0;i<8;i++)
	{
		UART1_TXBUFFER[16+2*i] = (ADC_Base0[i]>>8);
		UART1_TXBUFFER[16+2*i+1] = (ADC_Base0[i]&0xff);
	}
	UART1_Send_Data(UART1_TXBUFFER,32);

}
//-------------------------------------
void cmd_RGB888_565(void)
{
	u16 i,en;
	u32 f_addr,f_size;

	en = 0x00;
	if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==1)//¶þÎ¬ÂëÇø
	{
		FLASH2_GPIOSPI_Read (Addr_01min, str_buffer, 256);
		en = 0x00;
		f_addr = 0;
		f_size = 0;
		if(str_buffer[1]>10)
		{	
			str_buffer[1] = 10;	
		}

		for(i=0;i<str_buffer[1];i++)
		{
			if(str_buffer[0]!=frame_headerC)	
			{		break;	}			
			if(str_buffer[4+18*i+1]==UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX])//È·ÈÏÎÄ¼þºÅ
			{
				en = 0xff; 
				f_size = str_buffer[4+18*i+2];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+3];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+4];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+5];	

				f_addr = str_buffer[4+18*i+14];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+15];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+16];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+17];
				break;
			}
		}

		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  4;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];

		UART1_TXBUFFER[5] =  en;

		UART1_TXBUFFER[6] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[7] =  frame_last;
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));

		if(f_size>0)
		{
			//	display_flash_BMP (0,0,f_addr,2);
			RGB888_565(f_addr);
		}
	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==2)//ÎÄ×ÖÇø
	{
		FLASH2_GPIOSPI_Read (Addr_02min, str_buffer, 2048);
		en = 0x00;
		f_addr = 0;
		f_size = 0;
		if(str_buffer[1]>100)
		{	
			str_buffer[1] = 100;	
		}
		//		UART1_Send_Data(str_buffer,str_buffer[1]*18+6);

		for(i=0;i<str_buffer[1];i++)
		{
			if(str_buffer[0]!=frame_headerC)	
			{		break;	}			
			if(str_buffer[4+18*i+1]==UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX])//È·ÈÏÎÄ¼þºÅ
			{
				en = 0xff; 
				f_size = str_buffer[4+18*i+2];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+3];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+4];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+5];	

				f_addr = str_buffer[4+18*i+14];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+15];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+16];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+17];
				break;
			}
		}

		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  4;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];

		UART1_TXBUFFER[5] =  en;

		UART1_TXBUFFER[6] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[7] =  frame_last;
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
		if(f_size>0)
		{
			//	display_flash_BMP (0,0,f_addr,2);
			RGB888_565(f_addr);
		}

	}
	else if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX]==3)//²ÊÍ¼Çø
	{
		FLASH2_GPIOSPI_Read (Addr_03min, str_buffer, 256);
		en = 0x00;
		f_addr = 0;
		f_size = 0;
		if(str_buffer[1]>10)
		{	
			str_buffer[1] = 10;	
		}

		for(i=0;i<str_buffer[1];i++)
		{
			if(str_buffer[0]!=frame_headerC)	
			{		break;	}			
			if(str_buffer[4+18*i+1]==UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX])//È·ÈÏÎÄ¼þºÅ
			{
				en = 0xff; 
				f_size = str_buffer[4+18*i+2];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+3];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+4];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+5];	

				f_addr = str_buffer[4+18*i+14];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+15];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+16];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+17];
				break;
			}
		}

		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  4;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];

		UART1_TXBUFFER[5] =  en;

		UART1_TXBUFFER[6] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[7] =  frame_last;
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));

		if(f_size>0)
		{
			//	display_flash_BMP (0,0,f_addr,2);
			RGB888_565(f_addr);
		}
	}
	else
	{
		UART1_TXBUFFER[0] =  frame_headerD;
		UART1_TXBUFFER[1] =  4;  //len
		UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
		UART1_TXBUFFER[3] =  device.addr;
		UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];

		UART1_TXBUFFER[5] =  en;

		UART1_TXBUFFER[6] = 0;//check;
		for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
		{
			UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
		}
		UART1_TXBUFFER[7] =  frame_last;
		UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));
	}
}
//-------------------------------------
void RGB888_565(unsigned int addr)
{
	u32 i=0,j=0;//
	u32 offset,x_offset,y_offset;  //
	u32 x_wide;  //Í¼Æ¬Êµ¼ÊÈ¡µãµÄ¿í¶È ¡£Îª32µÄ±¶Êý
	//  u32 x_offset1;  //Êµ¼ÊÒªµÄ´°¿Ú³¤¿í
	u32 pix;  //ÏÔÊ¾µ½µÄ×Ó½ÚÊý
	unsigned int pix_temp;
	FLASH2_GPIOSPI_Read (addr, str_buffer, 64);
	if(str_buffer[28] ==0x18)   //24bit
	{
		offset = str_buffer[13];
		offset <<= 8;
		offset += str_buffer[12];
		offset <<= 8;
		offset += str_buffer[11];
		offset <<= 8;
		offset += str_buffer[10];

		x_offset = str_buffer[21];
		x_offset <<= 8;
		x_offset += str_buffer[20];
		x_offset <<= 8;
		x_offset += str_buffer[19];
		x_offset <<= 8;
		x_offset += str_buffer[18];

		if((x_offset%2)!=0)  //¹æ¶¨É¨ÃèÒ»ÐÐÊý¾Ý±ØÊÇ4×Ö½ÚµÄ±¶Êý£¬²»×ãµÄ²¹0
		{
			x_wide = x_offset + 2;
			x_wide &= ~0x01;			
		}
		else
		{
			x_wide = x_offset;
		}
		y_offset = str_buffer[25];
		y_offset <<= 8;
		y_offset += str_buffer[24];
		y_offset <<= 8;
		y_offset += str_buffer[23];
		y_offset <<= 8;
		y_offset += str_buffer[22];

		if((str_buffer[0]!='B')||(str_buffer[1]!='M'))  //ÄÚÈÝÈÏÎªÎÞÐ§
		{
			x_offset = 0;
		}
		if(y_offset>320)  //ÄÚÈÝÈÏÎªÎÞÐ§
		{
			x_offset = 0;
		}

		i = x_wide*y_offset*2+66;
		str_buffer[2] = (i&0xff);  			//SIZELL
		str_buffer[3] = ((i>>8)&0xff);  //SIZELH
		str_buffer[4] = ((i>>16)&0xff);  //SIZEHL
		str_buffer[5] = ((i>>24)&0xff);  //SIZEHH
		str_buffer[10] = 0x42;  //Æ«ÒÆÖµ
		str_buffer[28] = 0x10;  //rgb565
		str_buffer[30] = 0x03;  //ÓÐÑ¹Ëõ

		str_buffer[54] = 0x00;
		str_buffer[55] = 0xf8;
		str_buffer[56] = 0x00;
		str_buffer[57] = 0x00;
		str_buffer[58] = 0xe0;
		str_buffer[59] = 0x07;
		str_buffer[60] = 0x00;
		str_buffer[61] = 0x00;
		str_buffer[62] = 0x1f;
		str_buffer[63] = 0x00;
		str_buffer[64] = 0x00;
		str_buffer[65] = 0x00;
		UART1_Send_Data(str_buffer,66);

		if((x_offset>0)&&(x_offset<=240))
		{

			//			x_offset1 = x_offset;

			global_u16p = (u16*)str_buffer;  //±ãÓÃÈ¡Ò»´ÎÈ¡16Î»

			for(i=0;i<y_offset;i++)	
			{
				for(j=0;j<(x_offset);j++)	
				{
					pix = i*(x_wide)+j;  //ÏñËØ¼Ó1
					if(pix%1024==0)
					{
						if(pix>0)
						{
							UART1_Send_Data(str_buffer,2048);
						}
						FLASH2_GPIOSPI_Read (addr+offset+(pix*3), str_buffer, 3072);  //¶ÁÏñËØÄÚÈÝ
						pix_temp = pix;
					}

					// 				g16_temp = str_buffer[pix+1-pix_temp];	
					// 				g16_temp <<= 8;		
					// 				g16_temp += str_buffer[pix-pix_temp];		
					global_u16temp = (str_buffer[(pix-pix_temp)*3+2]>>3);
					global_u16temp <<= 6;
					global_u16temp += (str_buffer[(pix-pix_temp)*3+1]>>2);
					global_u16temp <<= 5;
					global_u16temp += (str_buffer[(pix-pix_temp)*3]>>3);

					global_u16p[pix-pix_temp]=global_u16temp;


				}
			}			 

			UART1_Send_Data(str_buffer,(pix-pix_temp+1)<<1);
		}
	}
}
void cmd_RGB_clear(void)
{
	u16 colour_t,x_l,y_l,x_o, y_o;
	if(UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX]>=9)
	{		 		 
		colour_t  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+6)&UART1_RX_MAX];
		colour_t <<= 8; 
		colour_t += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+7)&UART1_RX_MAX];

		x_l  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+8)&UART1_RX_MAX];
		x_l <<= 8; 
		x_l += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+9)&UART1_RX_MAX];
		y_l  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+10)&UART1_RX_MAX];
		y_l <<= 8; 
		y_l += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+11)&UART1_RX_MAX];

		x_o  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+12)&UART1_RX_MAX];
		x_o <<= 8; 
		x_o += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+13)&UART1_RX_MAX];
		y_o  = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+14)&UART1_RX_MAX];
		y_o <<= 8; 
		y_o += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+15)&UART1_RX_MAX];
		//void tft_Clear(u16 x,u16 y,u16 x_offset,u16 y_offset,u16 Color,u8 cs)
		tft_Clear(x_l,y_l,x_o,y_o,colour_t,UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+5)&UART1_RX_MAX]);		//		
	}
}
//-------------------------------------
void cmd_Save_ADC(void)
{
	u8 i,en;
	GPIO_SetBits(SSB0_PORT, SSB0_PIN);
	GPIO_SetBits(SSB1_PORT, SSB1_PIN);
	GPIO_SetBits(SSB2_PORT, SSB2_PIN);
	GPIO_SetBits(SSC0_PORT, SSC0_PIN);
	GPIO_SetBits(SSC1_PORT, SSC1_PIN);
	GPIO_SetBits(SSC2_PORT, SSC2_PIN);
	Delay_ms(200);	
	//»ñÈ¡»ùÏß
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

	FLASH2_GPIOSPI_SER(Addr_info);  ////Ã¿´Î²Á²Á4K
	FLASH2_GPIOSPI_Write(Addr_info, str_buffer, (str_buffer[1]<<1));

	en = 0xff;
	FLASH2_GPIOSPI_Read (Addr_info, str_buffer, 64);			
	for(i=0;i<16;i++)
	{
		if(str_buffer[6+i]!=global_u8p[i])
		{
			en = 0;
		}
	}

	UART1_TXBUFFER[0] =  frame_headerD;
	UART1_TXBUFFER[1] =  4;  //len
	UART1_TXBUFFER[2] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+3)&UART1_RX_MAX];
	UART1_TXBUFFER[3] =  device.addr;
	UART1_TXBUFFER[4] =  UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+4)&UART1_RX_MAX];
	UART1_TXBUFFER[5] =  en;

	UART1_TXBUFFER[6] = 0;//check;
	for(i=1;i<((UART1_TXBUFFER[1]<<1)-2);i++)
	{
		UART1_TXBUFFER[6] += UART1_TXBUFFER[i];
	}
	UART1_TXBUFFER[7] =  frame_last;
	UART1_Send_Data(UART1_TXBUFFER,(UART1_TXBUFFER[1]<<1));

}
//-----------------------------------------
void Dport_ChargeHB_ON (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*¿ªÆôLEDµÄÍâÉèÊ±ÖÓ*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE); 
	//-----------Êä³öÀà----------------
	/*ÉèÖÃÒý½ÅÄ£Ê½ÎªÍ¨ÓÃÍÆÍìÊä³ö*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*ÉèÖÃÒý½ÅËÙÂÊÎª50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*Ñ¡ÔñÒª¿ØÖÆµÄGPIOA×éÒý½Å*/															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, GPIO_Pin_12);
}
//-----------------------------------------
void Dport_ChargeHB_OFF (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*¿ªÆôLEDµÄÍâÉèÊ±ÖÓ*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE); 
	//-----------ÊäÈëÀà----------------
	/*ÉèÖÃÒý½ÅÄ£Ê½ÎªÍ¨ÓÃ¸¡¿ÕÊäÈë*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   

	/*ÉèÖÃÒý½ÅËÙÂÊÎª50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*Ñ¡ÔñÒª¿ØÖÆµÄGPIOA×éÒý½Å*/															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA, GPIO_Pin_12);

}
//-----------------------------------------
void Dport_ChargeHC_ON (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*¿ªÆôLEDµÄÍâÉèÊ±ÖÓ*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE); 
	//-----------Êä³öÀà----------------
	/*ÉèÖÃÒý½ÅÄ£Ê½ÎªÍ¨ÓÃÍÆÍìÊä³ö*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*ÉèÖÃÒý½ÅËÙÂÊÎª50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*Ñ¡ÔñÒª¿ØÖÆµÄGPIOA×éÒý½Å*/															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, GPIO_Pin_11);
}
//-----------------------------------------
void Dport_ChargeHC_OFF (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*¿ªÆôLEDµÄÍâÉèÊ±ÖÓ*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE); 
	//-----------ÊäÈëÀà----------------
	/*ÉèÖÃÒý½ÅÄ£Ê½ÎªÍ¨ÓÃ¸¡¿ÕÊäÈë*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   

	/*ÉèÖÃÒý½ÅËÙÂÊÎª50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*Ñ¡ÔñÒª¿ØÖÆµÄGPIOA×éÒý½Å*/															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA, GPIO_Pin_11);
}
//-----------------------------------------
u8 Frame_check_cmd1(void)
{
	u8 temp;
	u16 i,len;
	len = UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+1)&UART1_RX_MAX];
	len <<= 1;
	temp = 0;
	for(i=0;i<len-2;i++)
	{
		temp += UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+1+i)&UART1_RX_MAX];
	}
	if(temp==UART1_RXBUFFER[(UART1_RXBUFFE_HEAD+1+i)&UART1_RX_MAX])
	{
		temp = 0xff;    //Ð£ÑéÍ¨¹ý
	}
	else
	{
		temp = 0x00;
	}
	return temp;
}
//-----------------------------------------------------
void FiletoBuffer_ID(u8 area,u8 id,u8 *p)//ÒÔID·ÅÊ½µ÷¶ÁÎÄ¼þµ½BUFFER¡£
{
	u16 i;
	u32 f_addr,f_size;

	if(area==2)//ÎÄ×ÖÇø
	{
		FLASH2_GPIOSPI_Read (Addr_02min, str_buffer, 1024);
		f_addr = 0;
		f_size = 0;
		if(str_buffer[1]>52)
		{	
			str_buffer[1] = 52;	
		}

		for(i=0;i<str_buffer[1];i++)
		{
			if(str_buffer[0]!=frame_headerC)	
			{		break;	}			
			if(str_buffer[4+18*i+1]==id)//È·ÈÏÎÄ¼þºÅ
			{
				f_size = str_buffer[4+18*i+2];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+3];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+4];
				f_size <<= 8;
				f_size += str_buffer[4+18*i+5];	

				f_addr = str_buffer[4+18*i+14];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+15];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+16];
				f_addr <<= 8;
				f_addr += str_buffer[4+18*i+17];
				break;
			}
		}

		if(f_size>0)
		{
			FLASH2_GPIOSPI_Read (f_addr, p, f_size);
		}

	}
}

/**
 * @brief  Power on or Power off the specified USB port.
 * @param  usb_port: It can be 0, 1, 2, 3, 4 or 5 to select the USB port, use USB_ALL_INDEX will select all USB port.
 * @param  new_state: new state of the specified USB port.
 *   This parameter can be: USB_POWER_ON or USB_POWER_OFF.
 * @retval None
 */

u16 USB_GPIO_Pin[6] = {F3_EN_TPS54336_1_PIN,F1_EN_TPS54336_2_PIN,E6_EN_TPS54336_3_PIN,E0_EN_TPS54336_4_PIN,E2_EN_TPS54336_5_PIN,E4_EN_TPS54336_6_PIN};
GPIO_TypeDef* USB_GPIOx[6] = {GPIOF,GPIOF,GPIOE,GPIOE,GPIOE,GPIOE};

void usb_power_ctrl(u8 usb_port, u8 new_state)
{
	u8 i = 0;
	u8 i_min = usb_port;
	u8 i_max = usb_port;
	
	if(usb_port == USB_ALL_INDEX)
	{
		i_min = 0;
		i_max = 6;
	}
	
	if(new_state == USB_POWER_ON)
	{
		for(i=i_min;i<i_max;i++)
		{
			GPIO_ResetBits(USB_GPIOx[i],USB_GPIO_Pin[i]);
		}
	}
	else
	{
		for(i=i_min;i<i_max;i++)
		{
			GPIO_SetBits(USB_GPIOx[i],USB_GPIO_Pin[i]);
		}
	}
}

/**
 * @brief  Turn on or Turn off the specified LED.
 * @param  led_index: It can be LED_INDEX, LED1_INDEX or LED2_INDEX to select LED, use LED_ALL_INDEX will select all LED.
 * @param  new_state: new state of the specified LED.
 *   This parameter can be: LED_TURN_ON, LED_TURN_OFF or LED_TURN_NEGATION.
 * @retval None
 */

u16 LED_GPIO_Pin[3] = {G4_LED_PIN,G3_R_LED_PIN,G2_L_LED_PIN};
GPIO_TypeDef* LED_GPIOx[3] = {GPIOG,GPIOG,GPIOG};

void led_power_ctrl(u8 led_index, u8 new_state)
{
	u8 i = 0;
	u8 i_min = led_index;
	u8 i_max = led_index;
	
	if(led_index == LED_ALL_INDEX)
	{
		i_min = 0;
		i_max = 6;
	}
	
	if(new_state == LED_TURN_ON)
	{
		for(i=i_min;i<i_max;i++)
		{
			GPIO_ResetBits(LED_GPIOx[i],LED_GPIO_Pin[i]);
		}
	}
	else if(new_state == LED_TURN_OFF)
	{
		for(i=i_min;i<i_max;i++)
		{
			GPIO_SetBits(LED_GPIOx[i],LED_GPIO_Pin[i]);
		}
	}
	else
	{
		for(i=i_min;i<i_max;i++)
		{
			GPIO_NegationBits(LED_GPIOx[i],LED_GPIO_Pin[i]);
		}
	}
}

u16 HUB_SELECT_GPIO_Pin[4] = {B5_L_SEL0_PIN,B6_L_SEL1_PIN, G15_R_SEL0_PIN,B3_R_SEL1_PIN};
GPIO_TypeDef* HUB_SELECT_GPIOx[4] = {GPIOB,GPIOB,GPIOG,GPIOB};

static void hub_select(u8 lcd_index, u8 usb_index)
{
	u8 hub_select_mask = 0;
	if(usb_index < 6)
	{
		hub_select_mask = usb_index%3+1;
	}
	
	if(hub_select_mask & (0x01 << 0))
	{
		GPIO_SetBits(HUB_SELECT_GPIOx[lcd_index*2+0], HUB_SELECT_GPIO_Pin[lcd_index*2+0]);
	}
	else
	{
		GPIO_ResetBits(HUB_SELECT_GPIOx[lcd_index*2+0], HUB_SELECT_GPIO_Pin[lcd_index*2+0]);
	}

	if(hub_select_mask & (0x01 << 1))
	{
		GPIO_SetBits(HUB_SELECT_GPIOx[lcd_index*2+1], HUB_SELECT_GPIO_Pin[lcd_index*2+1]);
	}
	else
	{
		GPIO_ResetBits(HUB_SELECT_GPIOx[lcd_index*2+1], HUB_SELECT_GPIO_Pin[lcd_index*2+1]);
	}
}

void usb_mutually_exclusive_power_on(u8 lcd_index)
{
	u8 i = 0;
	u8 i_min = lcd_index*3;
	u8 i_max = lcd_index*3+3;
	u8 Uport_PowerUseTime_index = lcd_index;
	u8 usb_index = 0xff;
	u8 usb_power_on_position = 0;
	
	if(Uport_PowerUseTime[Uport_PowerUseTime_index]>0)
	{
		for(i=i_min;i<i_max;i++)
		{
			if((Dport_State[i]&0x0c)==0x0c)
			{
				usb_index = i;
			}
		}
	}
	
	if(usb_index < 6)
	{
		usb_power_on_position = 0x01<<usb_index;

		for(i=i_min;i<i_max;i++)
		{
			if((usb_power_on_position & (0x01<<i)) == 0)
			{
				usb_power_ctrl(i, USB_POWER_OFF);
			}
		}
		
		hub_select(lcd_index, usb_index);

		usb_power_ctrl(usb_index, USB_POWER_ON);
	}
	else
	{
		hub_select(lcd_index, usb_index);
		
		for(i=i_min;i<i_max;i++)
		{
			usb_power_ctrl(i, USB_POWER_OFF);
		}
	}
}



