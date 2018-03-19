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

#define SET_SECONDS_DEFAULT 0x00
#define SET_MINUTES_DEFAULT 0x00
#define SET_HOURS_DEFAULT 0x12
#define SET_ADDRESS_DEFAULT 0x50

static bool g_MasterCompletionFlag = false;
i2c_master_transfer_t masterXfer;
i2c_master_handle_t g_m_handle;
i2c_master_config_t masterConfig;

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

	for(Counter = 0; Counter < 3; Counter++){
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
	}
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

	//
	//	uint8_t read_data;
	//
	//	masterXfer.slaveAddress = 0x50;
	//	masterXfer.direction = kI2C_Read;
	//	masterXfer.subaddress = 0x02;
	//	masterXfer.subaddressSize = 1;
	//	masterXfer.data = &read_data;
	//	masterXfer.dataSize = 1;
	//	masterXfer.flags = kI2C_TransferDefaultFlag;
	//
	//	I2C_MasterTransferNonBlocking(I2C1, &g_m_handle,
	//			&masterXfer);
	//	while (!g_MasterCompletionFlag){}
	//	g_MasterCompletionFlag = false;
	//	PRINTF("%d",read_data);

}
void LCD_SetDate()
{

}
