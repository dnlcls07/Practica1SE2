/*
 * uart_tasks.c
 *
 *  Created on: Mar 26, 2018
 *      Author: dceli
 */

#include "uart_tasks.h"

void UART0_init_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;

	UART0_rx_semaphore = xSemaphoreCreateBinary();
	UART0_tx_semaphore = xSemaphoreCreateBinary();

	//Port configuration
	CLOCK_EnableClock ( kCLOCK_PortB );
	CLOCK_EnableClock ( kCLOCK_Uart0 );

	port_pin_config_t config_uart =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt3,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig ( PORTB, 16, &config_uart );
	PORT_SetPinConfig ( PORTB, 17, &config_uart );

	NVIC_EnableIRQ ( UART0_RX_TX_IRQn );
	NVIC_SetPriority ( UART0_RX_TX_IRQn, 8 );

	//UART configuration
	UART_GetDefaultConfig ( &uart_config );
	uart_config.baudRate_Bps = 115200U;
	uart_config.enableRx = pdTRUE;
	uart_config.enableTx = pdTRUE;
	UART_Init ( UART0, &uart_config, CLOCK_GetFreq ( UART0_CLK_SRC ) );
	UART_TransferCreateHandle ( UART0, &uart_pc_handle, UART0_UserCallback,
			NULL );

	xTaskCreate ( rx_task, "UART0_RXtask", configMINIMAL_STACK_SIZE,
			( void* ) cfg_struct, configMAX_PRIORITIES - 3, NULL );
	xTaskCreate ( tx_task, "UART0_TXtask", configMINIMAL_STACK_SIZE,
			( void* ) cfg_struct, configMAX_PRIORITIES - 3, NULL );
	vTaskDelay ( portMAX_DELAY );
}

//UART interrupt handler for reception
void UART0_UserCallback ( UART_Type *base, uart_handle_t *handle,
		status_t status, void *userData )
{
	BaseType_t xHigherPriorityTaskWoken;
	userData = userData;
	xHigherPriorityTaskWoken = pdFALSE;
	if (kStatus_UART_RxIdle == status)
	{
		xSemaphoreGiveFromISR( UART0_rx_semaphore, &xHigherPriorityTaskWoken );
	}
	if (kStatus_UART_TxIdle == status)
	{
		xSemaphoreGiveFromISR( UART0_tx_semaphore, &xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//Transmission task
void tx_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uart_pkg_struct_t * tx_queue_data;
	uint8_t tx_data;
	uart_transfer_t uart_echo_xfer;
	uart_echo_xfer.data = &tx_data;
	uart_echo_xfer.dataSize = sizeof(char);
	xSemaphoreGive( UART0_tx_semaphore );
	xSemaphoreGive( UART1_tx_semaphore );
	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_ENABLE, pdTRUE,
		pdTRUE,
		portMAX_DELAY );
		while ( uxQueueMessagesWaiting ( cfg_struct->tx_queue ) != pdFALSE )
		{
			xQueueReceive( cfg_struct->tx_queue, &tx_queue_data,
					portMAX_DELAY );
			tx_data = tx_queue_data->data;
			if (UART0 == tx_queue_data->uart_to_comm)
			{
				xSemaphoreTake( UART0_tx_semaphore, portMAX_DELAY );
			}
			else if (UART1 == tx_queue_data->uart_to_comm)
			{
				xSemaphoreTake( UART1_tx_semaphore, portMAX_DELAY );
			}
			UART_TransferSendNonBlocking ( tx_queue_data->uart_to_comm,
					tx_queue_data->uart_handle_to_comm, &uart_echo_xfer );
			if (UART0 == tx_queue_data->uart_to_comm)
			{
				xSemaphoreTake( UART0_tx_semaphore, portMAX_DELAY );
				xSemaphoreGive( UART0_tx_semaphore );
			}
			else if (UART1 == tx_queue_data->uart_to_comm)
			{
				xSemaphoreTake( UART1_tx_semaphore, portMAX_DELAY );
				xSemaphoreGive( UART1_tx_semaphore );
			}
			vPortFree ( tx_queue_data );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_DONE );
	}
}

void rx_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uart_pkg_struct_t * rx_config;
	uart_pkg_struct_t * rx_data_ptr;
	uint8_t rx_data;
	uart_transfer_t uart_rx_xfer;
	uart_rx_xfer.data = &rx_data;
	uart_rx_xfer.dataSize = sizeof(char);
	xSemaphoreGive( UART0_rx_semaphore );
	xSemaphoreGive( UART1_rx_semaphore );
	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_ENABLE, pdTRUE,
		pdTRUE,
		portMAX_DELAY );
		xQueueReceive( cfg_struct->rx_cfg_queue, &rx_config, portMAX_DELAY );
		while ( ( RETURN_CHAR != rx_data ) && ( ESC_CHAR != rx_data ) )
		{
			if (UART0 == rx_config->uart_to_comm)
			{
				xSemaphoreTake( UART0_rx_semaphore, portMAX_DELAY );
			}
			else if (UART1 == rx_config->uart_to_comm)
			{
				xSemaphoreTake( UART1_rx_semaphore, portMAX_DELAY );
			}
			UART_TransferReceiveNonBlocking ( rx_config->uart_to_comm,
					rx_config->uart_handle_to_comm, &uart_rx_xfer, NULL );
			if (UART0 == rx_config->uart_to_comm)
			{
				xSemaphoreTake( UART0_rx_semaphore, portMAX_DELAY );
				xSemaphoreGive( UART0_rx_semaphore );
			}
			else if (UART1 == rx_config->uart_to_comm)
			{
				xSemaphoreTake( UART1_rx_semaphore, portMAX_DELAY );
				xSemaphoreGive( UART1_rx_semaphore );
			}
			rx_data_ptr = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			rx_data_ptr->data = rx_data;
			rx_data_ptr->uart_handle_to_comm = cfg_struct->uart_handle;
			rx_data_ptr->uart_to_comm = UART0;
			if ( ( RETURN_CHAR != rx_data ) && ( ESC_CHAR != rx_data ))
			{
				xQueueSend( cfg_struct->rx_queue, &rx_data_ptr, portMAX_DELAY );
			}
		}
		vPortFree ( rx_config );
		if (ESC_CHAR == rx_data)
		{
			rx_data = 0;
			xEventGroupSetBits ( cfg_struct->menu_event_handle, ESC_RECEIVED );
		}
		rx_data = 0;
		xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_DONE );
	}
}

