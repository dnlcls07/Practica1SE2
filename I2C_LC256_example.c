#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
#include "tempFunction.h"

volatile bool g_MasterCompletionFlag = false;

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
        status_t status, void * userData)
{

	if (status == kStatus_Success)
	{
		g_MasterCompletionFlag = true;
	}
}

/*
 * @brief   Application entry point.
 */
int main(void)
{

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_I2c0);

	port_pin_config_t config_i2c =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt2,
	        kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTB, 2, &config_i2c);
	PORT_SetPinConfig(PORTB, 3, &config_i2c);

	i2c_master_config_t masterConfig;
	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 100000;
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	i2c_master_handle_t g_m_handle;
	I2C_MasterTransferCreateHandle(I2C0, &g_m_handle,
	        i2c_master_callback, NULL);

	i2c_master_transfer_t masterXfer;

	uint8_t data_buffer = 0x44;

	masterXfer.slaveAddress = 0x50;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0x05A0;
	masterXfer.subaddressSize = 2;
	masterXfer.data = &data_buffer;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0,  &g_m_handle,
	        &masterXfer);
	while (!g_MasterCompletionFlag){}
	g_MasterCompletionFlag = false;
	delay(60000);

	uint8_t read_data;

	masterXfer.slaveAddress = 0x50;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = 0x05A0;
	masterXfer.subaddressSize = 2;
	masterXfer.data = &read_data;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_m_handle,
	        &masterXfer);
	while (!g_MasterCompletionFlag){}
	g_MasterCompletionFlag = false;
	PRINTF("%d",&read_data);

	while (1)
	{
	}
	return 0;
}
