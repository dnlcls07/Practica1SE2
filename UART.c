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

#include "task.h"
#include "queue.h"
#include "semphr.h"

SemaphoreHandle_t tx_semaphore;
SemaphoreHandle_t rx_semaphore;
uart_handle_t uart_pc_handle;
uart_config_t uart_config;
uart_transfer_t uart_xfer;
void UART0_UserCallback ( UART_Type *base, uart_handle_t *handle,
		status_t status, void *userData );

//UART interrupt handler for reception
void UART0_UserCallback ( UART_Type *base, uart_handle_t *handle,
		status_t status, void *userData )
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( rx_semaphore, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//Transmission task
void tx_task ( void * arg )
{
	for ( ;; )
	{
//		xSemaphoreTake( &tx_semaphore, portMAX_DELAY );	//Wait for reception
		UART_TransferSendNonBlocking ( UART0, &uart_pc_handle, &uart_xfer );
	}
}

//Reception task
void rx_task ( void * arg )
{
	for ( ;; )
	{
//		xSemaphoreTake( &rx_semaphore, portMAX_DELAY );	//Wait for interrupt
		UART_TransferReceiveNonBlocking ( UART0, &uart_pc_handle, &uart_xfer,
		NULL );
//		xSemaphoreGive( tx_semaphore );	//Echo right after receiving
	}
}

int main ( void )
{

	BOARD_InitBootPins ();
	BOARD_InitBootClocks ();
	BOARD_InitBootPeripherals ();
	BOARD_InitDebugConsole ();

	//Port configuration
	CLOCK_EnableClock ( kCLOCK_PortB );
	CLOCK_EnableClock ( kCLOCK_Uart0 );

	port_pin_config_t config_uart =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt3,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig ( PORTB, 16, &config_uart );
	PORT_SetPinConfig ( PORTB, 17, &config_uart );

	//UART configuration
	UART_GetDefaultConfig ( &uart_config );
	uart_config.baudRate_Bps = 115200U;
	uart_config.enableRx = pdTRUE;
	uart_config.enableTx = pdTRUE;
	UART_Init ( UART0, &uart_config, CLOCK_GetFreq(UART0_CLK_SRC) );
	UART_TransferCreateHandle ( UART0, &uart_pc_handle, UART0_UserCallback,
	NULL );
	NVIC_EnableIRQ ( UART0_RX_TX_IRQn );
	NVIC_SetPriority ( UART0_RX_TX_IRQn, 7 );

	uart_xfer.data = NULL;
	uart_xfer.dataSize = sizeof(char);

	//Task startup
	xTaskCreate ( rx_task, "RXtask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 1, NULL );
	xTaskCreate ( tx_task, "TXtask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 2, NULL );
	vTaskStartScheduler ();

	while ( 1 )
	{

	}
	return 0;
}
