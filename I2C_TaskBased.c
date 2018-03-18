#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_uart.h"
#include "fsl_i2c.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"

SemaphoreHandle_t i2c_semaphore;
i2c_master_handle_t g_m_handle;

//UART interrupt handler for reception
void i2c_master_callback ( I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void *userData )
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	if (kStatus_Success == status)
	{
		xSemaphoreGiveFromISR( i2c_semaphore, xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//Transmission task
void i2c_write_task ( void * arg )
{
	for ( ;; )
	{
		i2c_master_transfer_t i2c_master_xfer;
		uint8_t data_buffer_out = 0x4D;
		i2c_master_xfer.slaveAddress = 0x50;
		i2c_master_xfer.direction = kI2C_Write;
		i2c_master_xfer.subaddress = 0x05A0;
		i2c_master_xfer.subaddressSize = 2;
		i2c_master_xfer.flags = kI2C_TransferDefaultFlag;
		i2c_master_xfer.data = &data_buffer_out;
		i2c_master_xfer.dataSize = sizeof ( data_buffer_out );
		I2C_MasterTransferNonBlocking ( I2C0, &g_m_handle, &i2c_master_xfer );
		vTaskDelay ( portMAX_DELAY );
	}
//	xSemaphoreTake( i2c_semaphore, portMAX_DELAY );
//	vTaskDelay ( pdMS_TO_TICKS( 500 ) );
//	i2c_master_xfer.direction = kI2C_Read;
//	i2c_master_xfer.data = &data_buffer_in;
//	I2C_MasterTransferNonBlocking ( I2C0, &g_m_handle, &i2c_master_xfer );
//	PRINTF ( "%d", &data_buffer_in );
}

void i2c_read_task ( void * arg )
{
	for ( ;; )
	{
		uint8_t data_buffer;
		xSemaphoreTake( i2c_semaphore, portMAX_DELAY );
		vTaskDelay ( pdMS_TO_TICKS( 100 ) );
		i2c_master_transfer_t i2c_master_xfer;
		i2c_master_xfer.slaveAddress = 0x50;
		i2c_master_xfer.direction = kI2C_Read;
		i2c_master_xfer.subaddress = 0x05A0;
		i2c_master_xfer.subaddressSize = 2;
		i2c_master_xfer.flags = kI2C_TransferDefaultFlag;
		i2c_master_xfer.data = &data_buffer;
		i2c_master_xfer.dataSize = sizeof ( data_buffer );
		I2C_MasterTransferNonBlocking ( I2C0, &g_m_handle, &i2c_master_xfer );
		xSemaphoreTake( i2c_semaphore, portMAX_DELAY );
		PRINTF ( "%d", &data_buffer );
	}
}

int main ( void )
{
	/* Init board hardware. */
	BOARD_InitBootPins ();
	BOARD_InitBootClocks ();
	BOARD_InitBootPeripherals ();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole ();

	CLOCK_EnableClock ( kCLOCK_PortB );
	CLOCK_EnableClock ( kCLOCK_I2c0 );

	port_pin_config_t config_i2c =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt2,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig ( PORTB, 2, &config_i2c );
	PORT_SetPinConfig ( PORTB, 3, &config_i2c );

	i2c_master_config_t masterConfig;
	I2C_MasterGetDefaultConfig ( &masterConfig );
	masterConfig.baudRate_Bps = 100000;
	I2C_MasterInit ( I2C0, &masterConfig, CLOCK_GetFreq ( kCLOCK_BusClk ) );
	I2C_MasterTransferCreateHandle ( I2C0, &g_m_handle, i2c_master_callback,
	NULL );
	NVIC_EnableIRQ ( I2C0_IRQn );
	NVIC_SetPriority ( I2C0_IRQn, 7 );

	//Task startup
	i2c_semaphore = xSemaphoreCreateBinary();

	xTaskCreate ( i2c_write_task, "I2C_MEM_W_Task", configMINIMAL_STACK_SIZE,
	NULL,
	configMAX_PRIORITIES - 2, NULL );
	xTaskCreate ( i2c_read_task, "I2C_MEM_R_TASK", configMINIMAL_STACK_SIZE,
	NULL, configMAX_PRIORITIES - 1, NULL );
	vTaskStartScheduler ();

	while ( 1 )
	{

	}
	return 0;
}

