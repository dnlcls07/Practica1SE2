/*
 * RTC_init.c
 *
 *  Created on: Mar 18, 2018
 *      Author: mango
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
#include "GlobalFunctions.h"
#include "RTC_init.h"
#include "LCDNokia5110.h"

#define SET_SECONDS_DEFAULT 0x00
#define SET_MINUTES_DEFAULT 0x59
#define SET_HOURS_DEFAULT 0x23
#define SET_ADDRESS_DEFAULT 0x50
#define YEAR_DEFAULT 0x2018

static bool g_MasterCompletionFlag = false;
i2c_master_transfer_t masterXfer;
i2c_master_handle_t g_m_handle;
i2c_master_config_t masterConfig;
uint8_t Time[5];
uint8_t LCD_Time[14];
uint16_t Year_LCD = YEAR_DEFAULT;
static uint8_t string1[] = "Hora";
static uint8_t string2[] = "Fecha";

void PORTC_IRQHandler()
{
	if(GPIO_PinRead(GPIOC, 5))
	{
		PORT_ClearPinsInterruptFlags(PORTC, 1<<5);
		PushB_time(0x04,1);
		//address
		//valor
	}
	if(GPIO_PinRead(GPIOC, 7))
	{
		PORT_ClearPinsInterruptFlags(PORTC, 1<<7);
		PushB_time(0x04,0);
	}
	if(GPIO_PinRead(GPIOC, 0))
	{
		PORT_ClearPinsInterruptFlags(PORTC, 1<<0);
		PushB_time(0x03,1);
	}
	if(GPIO_PinRead(GPIOC, 9))
	{
		PORT_ClearPinsInterruptFlags(PORTC, 1<<9);
		PushB_time(0x03,0);
	}
	if(GPIO_PinRead(GPIOC, 8))
	{
		PORT_ClearPinsInterruptFlags(PORTC, 1<<8);
		PushB_time(0x02,1);
	}
	if(GPIO_PinRead(GPIOC, 1))
	{
		PORT_ClearPinsInterruptFlags(PORTC, 1<<1);
		PushB_time(0x02,0);
	}
	//GPIO_TogglePinsOutput(GPIOB,1<<21);
	//BaseType_t xHigherPriorityTaskWoken;
	//PORT_ClearPinsInterruptFlags(PORTC, 1<<6);
	///xHigherPriorityTaskWoken = pdFALSE;
	//xSemaphoreGiveFromISR( counter_semaphore, &xHigherPriorityTaskWoken );
	//portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void * userData)
{

	if (status == kStatus_Success)
	{
		g_MasterCompletionFlag = true;
	}
}

void GetTime_default_t()
{
	uint8_t Counter = 0;
	uint8_t SubAddress = 0x02;
	uint8_t data_buffer = SET_SECONDS_DEFAULT;
	uint8_t buffer_0h = 0x08;
	g_MasterCompletionFlag = false;

	masterXfer.slaveAddress = SET_ADDRESS_DEFAULT;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0x00;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &buffer_0h;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C1,  &g_m_handle,
			&masterXfer);
	while (!g_MasterCompletionFlag){}
	g_MasterCompletionFlag = false;

	for(Counter = 0; Counter < 5; Counter++){
		masterXfer.slaveAddress = SET_ADDRESS_DEFAULT;
		masterXfer.direction = kI2C_Write;
		masterXfer.subaddress = SubAddress;
		masterXfer.subaddressSize = 1;
		masterXfer.data = &data_buffer;
		masterXfer.dataSize = 1;
		masterXfer.flags = kI2C_TransferDefaultFlag;

		I2C_MasterTransferNonBlocking(I2C1,  &g_m_handle,
				&masterXfer);
		while (!g_MasterCompletionFlag){}
		g_MasterCompletionFlag = false;
		SubAddress += 0x01;
		if(SubAddress == 0x03 )
		{
			data_buffer = SET_MINUTES_DEFAULT;
		}
		else if (SubAddress == 0x04)
		{
			data_buffer = SET_HOURS_DEFAULT;
		}
		else if (SubAddress == 0x05)    //year/date
		{
			data_buffer = 0x31;
		}
		else if (SubAddress == 0x06)       //mes
		{
			data_buffer = 0x12;
		}
	}
}

void PushB_time(uint8_t SubAddress, uint8_t cambio)
{
	uint8_t Read_data_push;
	masterXfer.slaveAddress = SET_ADDRESS_DEFAULT;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = SubAddress;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &Read_data_push;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;
	I2C_MasterTransferNonBlocking(I2C1, &g_m_handle,
			&masterXfer);

	while (!g_MasterCompletionFlag){}
	g_MasterCompletionFlag = false;

	if(cambio)
	{
		if(0x04 == SubAddress && Read_data_push == 0x23)
		{
			Read_data_push = 0x00;
		}
		else{
			Read_data_push += 0x01;
		}
	}
	else{
		Read_data_push -= 0x01;
	}
	//////////////////////////////////////////////
	masterXfer.slaveAddress = SET_ADDRESS_DEFAULT;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = SubAddress;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &Read_data_push;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;
	I2C_MasterTransferNonBlocking(I2C1,  &g_m_handle,
			&masterXfer);

	while (!g_MasterCompletionFlag){}
	g_MasterCompletionFlag = false;
}

void RTC_Init_t()
{
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_I2c1);

	port_pin_config_t config_i2c =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt2,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTC, 10, &config_i2c);
	PORT_SetPinConfig(PORTC, 11, &config_i2c);


	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 100000;
	I2C_MasterInit(I2C1, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));


	I2C_MasterTransferCreateHandle(I2C1, &g_m_handle,
			i2c_master_callback, NULL);
	/////////////////////////////////////////////////////////////////////////
	port_pin_config_t PushB =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTC, 5, &PushB); //setmas_horas
	PORT_SetPinConfig(PORTC, 7, &PushB); //setmenos_horas
	PORT_SetPinConfig(PORTC, 0, &PushB); //setmas_minutos
	PORT_SetPinConfig(PORTC, 9, &PushB); //setmenos_minutos
	PORT_SetPinConfig(PORTC, 8, &PushB); //setmas_segundos
	PORT_SetPinConfig(PORTC, 1, &PushB); //setmenos_segundos

	PORT_SetPinInterruptConfig(PORTC, 5, kPORT_InterruptRisingEdge);
	PORT_SetPinInterruptConfig(PORTC, 7, kPORT_InterruptRisingEdge);
	PORT_SetPinInterruptConfig(PORTC, 0, kPORT_InterruptRisingEdge);
	PORT_SetPinInterruptConfig(PORTC, 9, kPORT_InterruptRisingEdge);
	PORT_SetPinInterruptConfig(PORTC, 8, kPORT_InterruptRisingEdge);
	PORT_SetPinInterruptConfig(PORTC, 1, kPORT_InterruptRisingEdge);

	gpio_pin_config_t switch_config_gpio =
	{ kGPIO_DigitalInput, 1 };

	GPIO_PinInit(GPIOC, 5, &switch_config_gpio);
	GPIO_PinInit(GPIOC, 7, &switch_config_gpio);
	GPIO_PinInit(GPIOC, 0, &switch_config_gpio);
	GPIO_PinInit(GPIOC, 9, &switch_config_gpio);
	GPIO_PinInit(GPIOC, 8, &switch_config_gpio);
	GPIO_PinInit(GPIOC, 1, &switch_config_gpio);

	/////////////////////////////////////////////////////////////////////
	NVIC_EnableIRQ(PORTC_IRQn);
	NVIC_SetPriority(PORTC_IRQn,4);

}
void LCD_SetTime()
{
	uint8_t read_data;
	uint8_t Counter_time;
	uint8_t SubAddress = 0x02;
	for(Counter_time = 0; Counter_time <5; Counter_time++)
	{
		g_MasterCompletionFlag = false;
		masterXfer.slaveAddress = SET_ADDRESS_DEFAULT;
		masterXfer.direction = kI2C_Read;
		masterXfer.subaddress = SubAddress;
		masterXfer.subaddressSize = 1;
		masterXfer.data = &read_data;
		masterXfer.dataSize = 1;
		masterXfer.flags = kI2C_TransferDefaultFlag;
		I2C_MasterTransferNonBlocking(I2C1, &g_m_handle,
				&masterXfer);
		SubAddress += 0x01;
		while (!g_MasterCompletionFlag){}
		g_MasterCompletionFlag = false;
		Time[Counter_time] = read_data;
	}

	/////////////////////////////////////////////////////////
	//decodificadpres
	Decodificador_LCD(&Time[0],&LCD_Time[0] ,&LCD_Time[1]); //segundos
	Decodificador_LCD(&Time[1],&LCD_Time[2] ,&LCD_Time[3]); //minutos
	Decodificador_LCD(&Time[2],&LCD_Time[4] ,&LCD_Time[5]); //horas

	Decodificador_LCD(&Time[3],&LCD_Time[6] ,&LCD_Time[7]); //dia
	Decodificador_LCD(&Time[4],&LCD_Time[8] ,&LCD_Time[9]); //mes
	//	ConfigYear();
	Decodificador_LCD_year (Year_LCD, &LCD_Time[10] ,&LCD_Time[11],&LCD_Time[12] ,&LCD_Time[13]); //año
	///////////////////////////////////////////////////////

	//Reloj
	LCDNokia_gotoXY(26, 0); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendString(string1);
	LCDNokia_gotoXY(15, 1); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendChar(LCD_Time[4]); /*! It prints a character*/
	LCDNokia_gotoXY(21, 1);
	LCDNokia_sendChar(LCD_Time[5]);
	LCDNokia_gotoXY(28, 1);
	LCDNokia_sendChar(58);
	LCDNokia_gotoXY(34, 1); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendChar(LCD_Time[2]); /*! It prints a character*/
	LCDNokia_gotoXY(40, 1);
	LCDNokia_sendChar(LCD_Time[3]);
	LCDNokia_gotoXY(46, 1);
	LCDNokia_sendChar(58);
	LCDNokia_gotoXY(52, 1); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendChar(LCD_Time[0]); /*! It prints a character*/
	LCDNokia_gotoXY(58, 1);
	LCDNokia_sendChar(LCD_Time[1]);

	///////////////////////////////
	//Calendario
	//dd/mm/aaaa
	LCDNokia_gotoXY(22, 3); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendString(string2);
	LCDNokia_gotoXY(10, 4); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendChar(LCD_Time[6]); /*! It prints a character*/
	LCDNokia_gotoXY(16, 4);
	LCDNokia_sendChar(LCD_Time[7]);
	LCDNokia_gotoXY(22, 4);
	LCDNokia_sendChar(47);
	LCDNokia_gotoXY(28, 4); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendChar(LCD_Time[8]); /*! It prints a character*/
	LCDNokia_gotoXY(34, 4);
	LCDNokia_sendChar(LCD_Time[9]);
	LCDNokia_gotoXY(40, 4);
	LCDNokia_sendChar(47);
	LCDNokia_gotoXY(46, 4);
	LCDNokia_sendChar(LCD_Time[10]);
	LCDNokia_gotoXY(52, 4);
	LCDNokia_sendChar(LCD_Time[11]);
	LCDNokia_gotoXY(58, 4); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendChar(LCD_Time[12]); /*! It prints a character*/
	LCDNokia_gotoXY(64, 4);
	LCDNokia_sendChar(LCD_Time[13]);
}

void Decodificador_LCD (uint8_t *data1, uint8_t *data2, uint8_t *data3)
{
	if (*data1 <= 0x9){
		*data3 = *data1;
		*data2 = '0';
		*data3 += '0';
	}
	else{
		*data2 = *data1/0x10;
		*data3 = *data1%0x10;
		*data2 += '0';
		*data3 += '0';
	}
}

void Decodificador_LCD_year (uint16_t data1, uint8_t *data2, uint8_t *data3, uint8_t *data4, uint8_t *data5)
{
	uint8_t auxData = 0;
	if (data1 <= 0x0999)
	{
		*data3 = auxData/0x100;
		auxData = auxData%0x100;
		*data4 = auxData/0x10;
		*data5 = auxData%0x10;
		*data2 = '0';
		*data3 += '0';
		*data4 += '0';
		*data5 += '0';

	}
	else if(data1 <= 0x0099)
	{
		*data4 = data1/0x10;
		*data5 = data1%0x10;
		*data2 = '0';
		*data3 = '0';
		*data4 += '0';
		*data5 += '0';

	}
	else if(data1 <= 0x0009)
	{
		*data5 = data1;
		*data2 = '0';
		*data3 = '0';
		*data4 = '0';
		*data5 += '0';
	}
	else
	{
		*data2 = data1/0x1000;
		auxData = data1%0x1000;
		*data3 = auxData/0x100;
		auxData = auxData%0x100;
		*data4 = auxData/0x10;
		*data5 = auxData%0x10;

		*data2 += '0';
		*data3 += '0';
		*data4 += '0';
		*data5 += '0';
	}
}

void ConfigYear()
{
	if(('0' == LCD_Time[8]) &&  ('1' == LCD_Time[9]) )
	{
		if( ('0' == (LCD_Time[0])) && ('0' == (LCD_Time[1])) && ('0' == (LCD_Time[2])) && ('0' == (LCD_Time[3]))  )
		{

		}

	}

}

//
//uint8_t setYear(uint16 year)
//{
//	setLeapYear(year);
//	uint8_t auxData;
//	uint8_t auxMask = 0xC0;
//
//	if (year % 4 == 0) {
//		time->yearAndDate.yearAndDate.year = 0;
//		auxData = RTC_singleRead(0xA0, YEAR_DATE);
//		auxData &= ~(auxMask);
//		RTC_singleWrite(0xA0, YEAR_DATE, (av uxData | 0));
//
//		return 0;
//	}
//
//	year--;
//
//	if (year % 4 == 0) {
//		time->yearAndDate.yearAndDate.year = 1;
//		auxData = RTC_singleRead(0xA0, YEAR_DATE);
//		auxData &= ~(auxMask);
//
//		RTC_singleWrite(0xA0, YEAR_DATE, (auxData | 0x40));
//		return 1;
//	}
//
//	year--;
//
//	if (year % 4 == 0) {
//		time->yearAndDate.yearAndDate.year = 2;
//		auxData = RTC_singleRead(0xA0, YEAR_DATE);
//		auxData &= ~(auxMask);
//
//		RTC_singleWrite(0xA0, YEAR_DATE, (auxData | 0x80));
//		return 2;
//	}
//
//	year--;
//
//	if (year % 4 == 0) {
//		time->yearAndDate.yearAndDate.year = 3;
//		auxData = RTC_singleRead(0xA0, YEAR_DATE);
//		auxData &= ~(auxMask);
//		RTC_singleWrite(0xA0, YEAR_DATE, (auxData | 0xC0));
//
//		return 3;
//	}
//}
