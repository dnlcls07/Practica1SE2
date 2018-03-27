/*
 * i2c_tasks.c
 *
 *  Created on: Mar 26, 2018
 *      Author: dceli
 */

#include "i2c_tasks.h"

void i2c_init_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	port_pin_config_t config_i2c =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt2,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig ( PORTB, 2, &config_i2c );
	PORT_SetPinConfig ( PORTB, 3, &config_i2c );

	I2C_MasterGetDefaultConfig ( &masterConfig );
	masterConfig.baudRate_Bps = 100000;
	I2C_MasterInit ( I2C0, &masterConfig, CLOCK_GetFreq ( kCLOCK_BusClk ) );
	I2C_MasterTransferCreateHandle ( I2C0, &g_m_handle, i2c_master_callback,
			NULL );

	NVIC_EnableIRQ ( I2C0_IRQn );
	NVIC_SetPriority ( I2C0_IRQn, 7 );

	i2c_semaphore = xSemaphoreCreateBinary();

	xTaskCreate ( i2c_task, "I2CTask", configMINIMAL_STACK_SIZE,
			( void* ) cfg_struct, configMAX_PRIORITIES - 4, NULL );
}

void i2c_master_callback ( I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void *userData )
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	if (kStatus_Success == status)
	{
		xSemaphoreGiveFromISR( i2c_semaphore, &xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void i2c_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	i2c_master_transfer_t * i2c_ptr;
	xSemaphoreGive( i2c_semaphore );
	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_ENABLE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		xQueueReceive( cfg_struct->i2c_queue, &i2c_ptr, portMAX_DELAY );
		xSemaphoreTake( i2c_semaphore, portMAX_DELAY );
		I2C_MasterTransferNonBlocking ( I2C0, &g_m_handle, i2c_ptr );
		xSemaphoreTake( i2c_semaphore, portMAX_DELAY );
		xSemaphoreGive( i2c_semaphore );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_DONE );
	}
}
