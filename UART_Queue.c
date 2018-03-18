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
#include "queue.h"

#define RING_BUFFER_SIZE 1
#define RX_EMPTY (1<<0)
#define RX_nGOING (1<<1)
#define TX_ECHO_EN (1<<2)
#define TX_FULL (1<<3)

EventGroupHandle_t uart_event_handle;
uart_handle_t uart_pc_handle;
uart_config_t uart_config;
uint8_t rx_pc_RingBuffer [ RING_BUFFER_SIZE ] =
{ 0 };
size_t receivedBytes;

QueueHandle_t rx_tx_queue;

void UART0_UserCallback ( UART_Type *base, uart_handle_t *handle,
		status_t status, void *userData );

//UART interrupt handler for reception
void UART0_UserCallback ( UART_Type *base, uart_handle_t *handle,
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
		xEventGroupSetBitsFromISR ( uart_event_handle, RX_EMPTY,
				&xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//Transmission task
void tx_task ( void * arg )
{
	uint8_t tx_data;
	uart_transfer_t uart_echo_xfer;
	uart_echo_xfer.data = &tx_data;
	uart_echo_xfer.dataSize = sizeof(char);
	for ( ;; )
	{
		xEventGroupWaitBits ( uart_event_handle, TX_ECHO_EN, pdTRUE, pdTRUE,
		portMAX_DELAY );
		xQueueReceive( rx_tx_queue, &tx_data, portMAX_DELAY );
		UART_TransferSendNonBlocking ( UART0, &uart_pc_handle,
				&uart_echo_xfer );
		xEventGroupClearBits ( uart_event_handle, TX_ECHO_EN );
	}
}

void rx_task ( void * arg )
{
	uint8_t rx_data;
	uart_transfer_t uart_rx_xfer;
	uart_rx_xfer.data = &rx_data;
	uart_rx_xfer.dataSize = sizeof(char);
	xEventGroupSetBits ( uart_event_handle, RX_EMPTY | RX_nGOING );
	UART_TransferStartRingBuffer ( UART0, &uart_pc_handle, rx_pc_RingBuffer,
	RING_BUFFER_SIZE );
	for ( ;; )
	{
		xEventGroupWaitBits ( uart_event_handle, RX_EMPTY | RX_nGOING, pdFALSE,
		pdTRUE,
		portMAX_DELAY );
		UART_TransferReceiveNonBlocking ( UART0, &uart_pc_handle, &uart_rx_xfer,
		NULL );
		xEventGroupClearBits ( uart_event_handle, RX_nGOING );
		xEventGroupWaitBits ( uart_event_handle, RX_nGOING, pdFALSE, pdTRUE,
				portMAX_DELAY );
		xQueueSend( rx_tx_queue, &rx_data, portMAX_DELAY );
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
	UART_Init ( UART0, &uart_config, CLOCK_GetFreq ( UART0_CLK_SRC ) );
	UART_TransferCreateHandle ( UART0, &uart_pc_handle, UART0_UserCallback,
	NULL );
	NVIC_EnableIRQ ( UART0_RX_TX_IRQn );
	NVIC_SetPriority ( UART0_RX_TX_IRQn, 7 );
	//Task startup

	uart_event_handle = xEventGroupCreate ();
	rx_tx_queue = xQueueCreate( 1, sizeof(char) );

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

