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

#define RETURN_CHAR 0x0D

#define UART_BUFFER_SIZE 64
#define MEM_ADDR_SIZE 4

#define RX_ENABLE (1<<0)
#define RX_DONE (1<<1)
#define TX_ENABLE (1<<2)
#define TX_DONE (1<<3)

#define ADDR_ENABLE (1<<0)
#define ADDR_DONE (1<<1)
#define BCD_ENABLE (1<<2)
#define BCD_DONE (1<<3)

typedef struct
{
	UART_Type * uart_to_comm;
	uart_handle_t * uart_handle_to_comm;
	uint8_t data;
} uart_pkg_struct_t;

SemaphoreHandle_t tx_semaphore;
SemaphoreHandle_t rx_semaphore;

EventGroupHandle_t uart_event_handle;
EventGroupHandle_t menu_event_handle;

QueueHandle_t tx_queue;
QueueHandle_t rx_queue;
QueueHandle_t addr_queue;
QueueHandle_t bcd_queue;

uart_handle_t uart_pc_handle;
uart_config_t uart_config;

//UART interrupt handler for reception
void UART0_UserCallback ( UART_Type *base, uart_handle_t *handle,
		status_t status, void *userData )
{
	BaseType_t xHigherPriorityTaskWoken;
	userData = userData;
	xHigherPriorityTaskWoken = pdFALSE;
	if (kStatus_UART_RxIdle == status)
	{
		xSemaphoreGiveFromISR( rx_semaphore, &xHigherPriorityTaskWoken );
	}
	if (kStatus_UART_TxIdle == status)
	{
		xSemaphoreGiveFromISR( tx_semaphore, &xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void addr_parser_task ( void * arg )
{
	uart_pkg_struct_t * uart_pkg;
	uint8_t msg_cnt;
	uint8_t addr [ 4 ];
	uint16_t relative_addr;
	for ( ;; )
	{
		xEventGroupWaitBits ( menu_event_handle, ADDR_ENABLE, pdFALSE, pdTRUE,
		portMAX_DELAY );
		for ( msg_cnt = 0; msg_cnt <= 3; msg_cnt++ )
		{
			xQueueReceive( rx_queue, &uart_pkg, portMAX_DELAY );
			addr [ msg_cnt ] = uart_pkg->data;
			vPortFree ( uart_pkg );
		}
		if (addr [ 0 ] >= 65)	//If the data is A-F, set to its value
			relative_addr = ( addr [ 0 ] - '7' ) * 4096;//Multiply by 16^3 to set the equivalent value according to the position
		else
			relative_addr = ( addr [ 0 ] - '0' ) * 4096;//If not, the data is 0-9
		if (addr [ 1 ] >= 65)
			relative_addr += ( addr [ 1 ] - '7' ) * 256;	//Multiply by 16^2
		else
			relative_addr += ( addr [ 1 ] - '0' ) * 256;
		if (addr [ 2 ] >= 65)
			relative_addr += ( addr [ 2 ] - '7' ) * 16;	//Multiply by 16^1
		else
			relative_addr += ( addr [ 2 ] - '0' ) * 16;
		if (addr [ 3 ] >= 65)
			relative_addr += addr [ 3 ] - '7';			//Multiply by 16^0
		else
			relative_addr += addr [ 3 ] - '0';
		xQueueSend( addr_queue, &relative_addr, portMAX_DELAY );
		xEventGroupSetBits ( menu_event_handle, ADDR_DONE );
	}
}

void bcd_parser_task ( void * arg )
{
	uint8_t bytesReceived;
	uint8_t bcd_data [ 2 ];
	uart_pkg_struct_t * uart_pkg;
	uint16_t parsed_data;
	for ( ;; )
	{
		xEventGroupWaitBits ( menu_event_handle, BCD_ENABLE, pdFALSE, pdTRUE,
		portMAX_DELAY );
		for ( bytesReceived = 0; uxQueueMessagesWaiting ( rx_queue ) != pdFALSE;
				bytesReceived++ )
		{
			xQueueReceive( rx_queue, &uart_pkg, portMAX_DELAY );
			bcd_data [ bytesReceived ] = uart_pkg->data;
			vPortFree ( uart_pkg );
		}
		parsed_data = ( ( bcd_data [ 0 ] - '0' ) * 10 )
				+ ( bcd_data [ 1 ] - '0' );
		xQueueSend( bcd_queue, &parsed_data, portMAX_DELAY );
		xEventGroupSetBits ( menu_event_handle, BCD_DONE );
	}
}

void read_sequence_task ( void * arg )
{
	uint8_t msg_cnt;
	uint8_t read_msg [ ] = "\n\rRead address:\n\r\0";
	uint8_t size_msg [ ] = "\n\rRead size:\n\r\0";
	uart_pkg_struct_t * uart_pkg;
	for ( msg_cnt = 0; read_msg [ msg_cnt ] != '\0'; msg_cnt++ )
	{
		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = read_msg [ msg_cnt ];
		uart_pkg->uart_handle_to_comm = &uart_pc_handle;
		uart_pkg->uart_to_comm = UART0;
		xQueueSend( tx_queue, &uart_pkg, portMAX_DELAY );
	}
	xEventGroupSetBits ( uart_event_handle, TX_ENABLE );
	xEventGroupWaitBits ( uart_event_handle, TX_ENABLE | TX_DONE, pdTRUE,
	pdTRUE, portMAX_DELAY );
	xQueueSend( rx_queue, &uart_pkg, portMAX_DELAY );
	xEventGroupSetBits ( uart_event_handle, RX_ENABLE );
	xEventGroupWaitBits ( uart_event_handle, RX_ENABLE | RX_DONE, pdTRUE,
	pdTRUE, portMAX_DELAY );
	xEventGroupSetBits ( menu_event_handle, ADDR_ENABLE );
	xEventGroupWaitBits ( menu_event_handle, ADDR_ENABLE | ADDR_DONE, pdTRUE,
	pdTRUE, portMAX_DELAY );
	for ( msg_cnt = 0; size_msg [ msg_cnt ] != '\0'; msg_cnt++ )
	{
		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = size_msg [ msg_cnt ];
		xQueueSend( tx_queue, &uart_pkg, portMAX_DELAY );
	}
	xEventGroupSetBits ( uart_event_handle, TX_ENABLE );
	xEventGroupWaitBits ( uart_event_handle, TX_ENABLE | TX_DONE, pdTRUE,
	pdTRUE, portMAX_DELAY );
	xQueueSend( rx_queue, &uart_pkg, portMAX_DELAY );
	xEventGroupSetBits ( uart_event_handle, RX_ENABLE );
	xEventGroupWaitBits ( uart_event_handle, RX_ENABLE | RX_DONE, pdTRUE,
	pdTRUE, portMAX_DELAY );
	xEventGroupSetBits ( menu_event_handle, BCD_ENABLE );
	xEventGroupWaitBits ( menu_event_handle, BCD_ENABLE | BCD_DONE, pdTRUE,
	pdTRUE, portMAX_DELAY );
}

//Transmission task
void tx_task ( void * arg )
{
	uart_pkg_struct_t * tx_queue_data;
	uint8_t tx_data;
	uart_transfer_t uart_echo_xfer;
	uart_echo_xfer.data = &tx_data;
	uart_echo_xfer.dataSize = sizeof(char);
	xSemaphoreGive( tx_semaphore );
	for ( ;; )
	{
		xEventGroupWaitBits ( uart_event_handle, TX_ENABLE, pdFALSE, pdTRUE,
		portMAX_DELAY );
		while ( uxQueueMessagesWaiting ( tx_queue ) != pdFALSE )
		{
			xQueueReceive( tx_queue, &tx_queue_data, portMAX_DELAY );
			tx_data = tx_queue_data->data;
			xSemaphoreTake( tx_semaphore, portMAX_DELAY );
			UART_TransferSendNonBlocking ( tx_queue_data->uart_to_comm,
					tx_queue_data->uart_handle_to_comm, &uart_echo_xfer );
//			UART_TransferSendNonBlocking ( UART0, &uart_pc_handle,
//					&uart_echo_xfer );
			xSemaphoreTake( tx_semaphore, portMAX_DELAY );
			xSemaphoreGive( tx_semaphore );
			vPortFree ( tx_queue_data );
		}
		xEventGroupSetBits ( uart_event_handle, TX_DONE );
	}
}

void rx_task ( void * arg )
{
	uart_pkg_struct_t * rx_config;
	uart_pkg_struct_t * rx_data_ptr;
	uint8_t rx_data;
	uart_transfer_t uart_rx_xfer;
	uart_rx_xfer.data = &rx_data;
	uart_rx_xfer.dataSize = sizeof(char);
	xSemaphoreGive( rx_semaphore );
	for ( ;; )
	{
		xEventGroupWaitBits ( uart_event_handle, RX_ENABLE, pdFALSE,
		pdTRUE,
		portMAX_DELAY );
		xQueueReceive( rx_queue, &rx_config, portMAX_DELAY );
		while ( RETURN_CHAR != rx_data )
		{
			xSemaphoreTake( rx_semaphore, portMAX_DELAY );
			UART_TransferReceiveNonBlocking ( rx_config->uart_to_comm,
					rx_config->uart_handle_to_comm, &uart_rx_xfer, NULL );
//			UART_TransferReceiveNonBlocking ( UART0, &uart_pc_handle,
//					&uart_rx_xfer, NULL );
			xSemaphoreTake( rx_semaphore, portMAX_DELAY );
			xSemaphoreGive( rx_semaphore );
			rx_data_ptr = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			rx_data_ptr->data = rx_data;
			rx_data_ptr->uart_handle_to_comm = &uart_pc_handle;
			rx_data_ptr->uart_to_comm = UART0;
			if ( RETURN_CHAR != rx_data)
				xQueueSend( rx_queue, &rx_data_ptr, portMAX_DELAY );
		}
//		vPortFree ( rx_config );
		rx_data = 0;
		xEventGroupSetBits ( uart_event_handle, RX_DONE );
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
	NVIC_SetPriority ( UART0_RX_TX_IRQn, 8 );
	;

	tx_semaphore = xSemaphoreCreateBinary();
	rx_semaphore = xSemaphoreCreateBinary();

	uart_event_handle = xEventGroupCreate ();
	menu_event_handle = xEventGroupCreate ();

	addr_queue = xQueueCreate( 1, sizeof(uint16_t) );
	bcd_queue = xQueueCreate( 1, sizeof(uint16_t) );
	rx_queue = xQueueCreate( UART_BUFFER_SIZE, sizeof(void*) );
	tx_queue = xQueueCreate( UART_BUFFER_SIZE, sizeof(void*) );

//Task startup

	xTaskCreate ( rx_task, "RXtask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 3, NULL );
	xTaskCreate ( tx_task, "TXtask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 2, NULL );
	xTaskCreate ( read_sequence_task, "RDtask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 1, NULL );
	xTaskCreate ( addr_parser_task, "ADDRtask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 4, NULL );
	xTaskCreate ( bcd_parser_task, "BCDTask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 5, NULL );
	vTaskStartScheduler ();

	while ( 1 )
	{

	}
	return 0;
}

