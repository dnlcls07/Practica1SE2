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
#include "event_groups.h"

#define RING_BUFFER_SIZE 1
#define RX_EMPTY (1<<0)
#define RX_nGOING (1<<1)
#define TX_ECHO_EN (1<<2)
#define TX_FULL (1<<3)

EventGroupHandle_t uart_event_handle;
uart_handle_t uart_pc_handle;
uart_config_t uart_config;
uart_transfer_t uart_rx_xfer;
uart_transfer_t uart_echo_xfer;
uint8_t rx_pc_RingBuffer [ RING_BUFFER_SIZE ] =
{ 0 };
size_t receivedBytes;

uint8_t rx_data;
uint8_t tx_data;

void UART1_UserCallback ( UART_Type *base, uart_handle_t *handle,
		status_t status, void *userData );

//UART interrupt handler for reception
void UART1_UserCallback ( UART_Type *base, uart_handle_t *handle,
		status_t status, void *userData )
{
	BaseType_t xHigherPriorityTaskWoken;
	userData = userData;
	xHigherPriorityTaskWoken = pdFALSE;
	if (kStatus_UART_RxIdle == status)
	{
		xEventGroupClearBitsFromISR ( uart_event_handle, RX_EMPTY );
		xEventGroupSetBitsFromISR ( uart_event_handle, RX_nGOING | TX_ECHO_EN,
				&xHigherPriorityTaskWoken );
	}
	if (kStatus_UART_TxIdle == status)
	{
		xEventGroupSetBitsFromISR(uart_event_handle, RX_EMPTY, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//Transmission task
void tx_task ( void * arg )
{
	for ( ;; )
	{
		xEventGroupWaitBits ( uart_event_handle, TX_ECHO_EN, pdTRUE, pdTRUE,
		portMAX_DELAY );
		tx_data = rx_data;
		UART_TransferSendNonBlocking ( UART1, &uart_pc_handle,
				&uart_echo_xfer );
		xEventGroupClearBits(uart_event_handle, TX_ECHO_EN);
	}
}

void rx_task ( void*arg )
{
	xEventGroupSetBits ( uart_event_handle, RX_EMPTY | RX_nGOING );
	UART_TransferStartRingBuffer ( UART1, &uart_pc_handle, rx_pc_RingBuffer,
	RING_BUFFER_SIZE );
	for ( ;; )
	{
		xEventGroupWaitBits ( uart_event_handle, RX_EMPTY | RX_nGOING, pdFALSE,
		pdTRUE,
		portMAX_DELAY );
		UART_TransferReceiveNonBlocking ( UART1, &uart_pc_handle, &uart_rx_xfer,
				NULL );
		xEventGroupClearBits ( uart_event_handle, RX_nGOING );
	}
}

int main ( void )
{

	BOARD_InitBootPins ();
	BOARD_InitBootClocks ();
	BOARD_InitBootPeripherals ();
	BOARD_InitDebugConsole ();

	//Port configuration
	CLOCK_EnableClock ( kCLOCK_PortC );
	CLOCK_EnableClock ( kCLOCK_Uart1 );

	port_pin_config_t config_uart =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt3,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig ( PORTC, 3, &config_uart );
	PORT_SetPinConfig ( PORTC, 4, &config_uart );

	//UART configuration
	UART_GetDefaultConfig ( &uart_config );
	uart_config.baudRate_Bps = 9600U;
	uart_config.enableRx = pdTRUE;
	uart_config.enableTx = pdTRUE;
	UART_Init ( UART1, &uart_config, CLOCK_GetFreq ( UART1_CLK_SRC ) );
	UART_TransferCreateHandle ( UART1, &uart_pc_handle, UART1_UserCallback,
	NULL );
	NVIC_EnableIRQ ( UART1_RX_TX_IRQn );
	NVIC_SetPriority ( UART1_RX_TX_IRQn, 7 );

	uart_rx_xfer.data = &rx_data;
	uart_rx_xfer.dataSize = sizeof(char);

	uart_echo_xfer.data = &tx_data;
	uart_echo_xfer.dataSize = sizeof(char);

	//Task startup

	uart_event_handle = xEventGroupCreate ();
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

