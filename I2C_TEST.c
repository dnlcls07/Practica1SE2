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
#define ESC_CHAR 0x1B
#define ALL_EVENTS 0x7FFFFF

#define UART_BUFFER_SIZE 64
#define MAX_I2C_PAGE_SIZE 64
#define MEM_ADDR_SIZE 4

#define RX_ENABLE (1<<0)
#define RX_DONE (1<<1)
#define TX_ENABLE (1<<2)
#define TX_DONE (1<<3)

#define ADDR_ENABLE (1<<0)
#define ADDR_DONE (1<<1)
#define BCD_ENABLE (1<<2)
#define BCD_DONE (1<<3)

#define READ_SEQ_ENABLE (1<<4)
#define READ_SEQ_DONE (1<<5)
#define WRITE_SEQ_ENABLE (1<<6)
#define WRITE_SEQ_DONE (1<<7)
#define ESC_RECEIVED (1<<8)
#define ESC_B2MENU (1<<9)

#define I2C_ENABLE (1<<0)
#define I2C_DONE (1<<1)

#define NO_QUEUE 1

typedef struct
{
	UART_Type * uart_to_comm;
	uart_handle_t * uart_handle_to_comm;
	uint8_t data;
} uart_pkg_struct_t;

SemaphoreHandle_t tx_semaphore;
SemaphoreHandle_t rx_semaphore;
SemaphoreHandle_t i2c_semaphore;
SemaphoreHandle_t i2c_mutex;

EventGroupHandle_t uart_event_handle;
EventGroupHandle_t menu_event_handle;
EventGroupHandle_t i2c_event_handle;

QueueHandle_t tx_queue;
QueueHandle_t rx_queue;
QueueHandle_t rx_cfg_queue;
QueueHandle_t addr_queue;
QueueHandle_t bcd_queue;
QueueHandle_t i2c_queue;

uart_handle_t uart_pc_handle;
uart_config_t uart_config;
i2c_master_handle_t g_m_handle;

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
	i2c_master_transfer_t * i2c_ptr;
	xSemaphoreGive( i2c_semaphore );
	for ( ;; )
	{
		xEventGroupWaitBits ( i2c_event_handle, I2C_ENABLE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		xQueueReceive( i2c_queue, &i2c_ptr, portMAX_DELAY );
		xSemaphoreTake( i2c_semaphore, portMAX_DELAY );
		xSemaphoreTake( i2c_mutex, portMAX_DELAY );
		I2C_MasterTransferNonBlocking ( I2C0, &g_m_handle, i2c_ptr );
		xSemaphoreTake( i2c_semaphore, portMAX_DELAY );
		xSemaphoreGive( i2c_mutex );
		xSemaphoreGive( i2c_semaphore );
		xEventGroupSetBits ( i2c_event_handle, I2C_DONE );
	}
}

void addr_parser_task ( void * arg )
{
	uart_pkg_struct_t * uart_pkg;
	uint8_t msg_cnt;
	uint8_t addr [ 4 ];
	uint16_t relative_addr;
	for ( ;; )
	{
		xEventGroupWaitBits ( menu_event_handle, ADDR_ENABLE, pdTRUE,
		pdTRUE,
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
		xEventGroupWaitBits ( menu_event_handle, BCD_ENABLE, pdTRUE, pdTRUE,
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

void esc_sequence_task ( void * arg )
{
	uart_pkg_struct_t * dummy_pkg;
	for ( ;; )
	{
		xEventGroupWaitBits ( menu_event_handle, ESC_RECEIVED, pdTRUE, pdTRUE,
		portMAX_DELAY );
#ifdef NO_QUEUE
		xEventGroupClearBits ( uart_event_handle, RX_ENABLE );
#endif
#ifdef CLEAR_QUEUE
		xEventGroupClearBits ( uart_event_handle, ALL_EVENTS );
		xEventGroupClearBits ( i2c_event_handle, ALL_EVENTS );
		xEventGroupClearBits ( menu_event_handle, ALL_EVENTS );
		while ( uxQueueMessagesWaiting ( rx_queue ) != pdFALSE )
		{
			xQueueReceive( rx_queue, &dummy_pkg, portMAX_DELAY );
			vPortFree ( dummy_pkg );
		}
#endif
#ifdef DELETE_QUEUE
		xEventGroupClearBits ( uart_event_handle, ALL_EVENTS );
		xEventGroupClearBits ( i2c_event_handle, ALL_EVENTS );
		xEventGroupClearBits ( menu_event_handle, ALL_EVENTS );
		vQueueDelete(rx_queue);
		rx_queue = xQueueCreate( UART_BUFFER_SIZE, sizeof(void*) );
#endif
		xEventGroupSetBits ( menu_event_handle, ESC_B2MENU );
	}
}

void menu_sequence_task ( void * arg )
{
	uint8_t menu_msg [ ] = "\n\rChoose a menu:\n\r1)Write\n\r2)Read\n\r\0";
	uart_pkg_struct_t * uart_pkg;
	uint8_t msg_cnt;
	uint32_t waitForSeq;
	for ( ;; )
	{
		for ( msg_cnt = 0; menu_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = menu_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = &uart_pc_handle;
			uart_pkg->uart_to_comm = UART0;
			xQueueSend( tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( uart_event_handle, TX_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = &uart_pc_handle;
		uart_pkg->uart_to_comm = UART0;
		xQueueSend( rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( uart_event_handle, RX_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		xQueueReceive( rx_queue, &uart_pkg, portMAX_DELAY );
		switch ( uart_pkg->data )
		{
			case '1' :
				xEventGroupSetBits ( menu_event_handle, WRITE_SEQ_ENABLE );
				waitForSeq = WRITE_SEQ_DONE;
				break;
			case '2' :
				xEventGroupSetBits ( menu_event_handle, READ_SEQ_ENABLE );
				waitForSeq = READ_SEQ_DONE;
				break;
			default :
				break;
		}
		xEventGroupWaitBits ( menu_event_handle, waitForSeq | ESC_B2MENU,
		pdTRUE,
		pdFALSE, portMAX_DELAY );
	}
}

void read_sequence_task ( void * arg )
{
	i2c_master_transfer_t * i2c_xfer_ptr;
	uint8_t msg_cnt;
	uint8_t read_msg [ ] = "\n\rRead address:\n\r\0";
	uint8_t size_msg [ ] = "\n\rRead size:\n\r\0";

	uint16_t addr;
	uint8_t data_size;

	uart_pkg_struct_t * uart_pkg;
	uart_pkg_struct_t * i2c_pkg;
	for ( ;; )
	{

		xEventGroupWaitBits ( menu_event_handle, READ_SEQ_ENABLE, pdTRUE,
		pdTRUE, portMAX_DELAY );

		for ( msg_cnt = 0; read_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = read_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = &uart_pc_handle;
			uart_pkg->uart_to_comm = UART0;
			xQueueSend( tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( uart_event_handle, TX_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = &uart_pc_handle;
		uart_pkg->uart_to_comm = UART0;
		xQueueSend( rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( uart_event_handle, RX_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		xEventGroupSetBits ( menu_event_handle, ADDR_ENABLE );
		xEventGroupWaitBits ( menu_event_handle, ADDR_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		for ( msg_cnt = 0; size_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = size_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = &uart_pc_handle;
			uart_pkg->uart_to_comm = UART0;
			xQueueSend( tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( uart_event_handle, TX_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = &uart_pc_handle;
		uart_pkg->uart_to_comm = UART0;
		xQueueSend( rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( uart_event_handle, RX_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		xEventGroupSetBits ( menu_event_handle, BCD_ENABLE );
		xEventGroupWaitBits ( menu_event_handle, BCD_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );

		xQueueReceive( addr_queue, &addr, portMAX_DELAY );
		xQueueReceive( bcd_queue, &data_size, portMAX_DELAY );
		uint8_t i2c_received_data [ data_size ];

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Read;
		i2c_xfer_ptr->subaddress = addr;
		i2c_xfer_ptr->subaddressSize = 2;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &i2c_received_data [ 0 ];
		i2c_xfer_ptr->dataSize = data_size;

		xQueueSend( i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( i2c_event_handle, I2C_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		for ( msg_cnt = 0; msg_cnt < data_size; msg_cnt++ )
		{
			i2c_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			i2c_pkg->uart_handle_to_comm = &uart_pc_handle;
			i2c_pkg->uart_to_comm = UART0;
			i2c_pkg->data = i2c_received_data [ msg_cnt ];
			xQueueSend( tx_queue, &i2c_pkg, portMAX_DELAY );
		}
		vPortFree ( i2c_xfer_ptr );
		xEventGroupSetBits ( uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( uart_event_handle, TX_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		xEventGroupSetBits ( menu_event_handle, READ_SEQ_DONE );
	}
}

void write_sequence_task ( void * arg )
{
	i2c_master_transfer_t * i2c_xfer_ptr;
	uint8_t msg_cnt;
	uint8_t msg_size;
	uint16_t addr;
	uint8_t read_msg [ ] = "\n\rWrite address:\n\r\0";
	uint8_t msg_msg [ ] = "\n\rWrite message:\n\r\0";
	uart_pkg_struct_t * uart_pkg;
	for ( ;; )
	{
		xEventGroupWaitBits ( menu_event_handle, WRITE_SEQ_ENABLE, pdTRUE,
		pdTRUE, portMAX_DELAY );
		for ( msg_cnt = 0; read_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = read_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = &uart_pc_handle;
			uart_pkg->uart_to_comm = UART0;
			xQueueSend( tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( uart_event_handle, TX_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = &uart_pc_handle;
		uart_pkg->uart_to_comm = UART0;
		xQueueSend( rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( uart_event_handle, RX_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		xEventGroupSetBits ( menu_event_handle, ADDR_ENABLE );
		xEventGroupWaitBits ( menu_event_handle, ADDR_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		for ( msg_cnt = 0; msg_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = msg_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = &uart_pc_handle;
			uart_pkg->uart_to_comm = UART0;
			xQueueSend( tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( uart_event_handle, TX_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );

		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = &uart_pc_handle;
		uart_pkg->uart_to_comm = UART0;

		xQueueSend( rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( uart_event_handle, RX_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );

		xQueueReceive( addr_queue, &addr, portMAX_DELAY );
		msg_size = uxQueueMessagesWaiting ( rx_queue );
		uint8_t msg_received [ msg_size ];
		for ( msg_cnt = 0; msg_cnt < msg_size; msg_cnt++ )
		{
			xQueueReceive( rx_queue, &msg_received [ msg_cnt ], portMAX_DELAY );
		}
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Write;
		i2c_xfer_ptr->subaddress = addr;
		i2c_xfer_ptr->subaddressSize = 2;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &msg_received [ 0 ];
		i2c_xfer_ptr->dataSize = msg_size;
		xQueueSend( i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( i2c_event_handle, I2C_DONE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		xEventGroupSetBits ( menu_event_handle, WRITE_SEQ_DONE );
	}
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
		xEventGroupWaitBits ( uart_event_handle, TX_ENABLE, pdTRUE, pdTRUE,
		portMAX_DELAY );
		while ( uxQueueMessagesWaiting ( tx_queue ) != pdFALSE )
		{
			xQueueReceive( tx_queue, &tx_queue_data, portMAX_DELAY );
			tx_data = tx_queue_data->data;
			xSemaphoreTake( tx_semaphore, portMAX_DELAY );
			UART_TransferSendNonBlocking ( tx_queue_data->uart_to_comm,
					tx_queue_data->uart_handle_to_comm, &uart_echo_xfer );
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
		xEventGroupWaitBits ( uart_event_handle, RX_ENABLE, pdTRUE,
		pdTRUE,
		portMAX_DELAY );
		xQueueReceive( rx_cfg_queue, &rx_config, portMAX_DELAY );
		while ( ( RETURN_CHAR != rx_data ) && ( ESC_CHAR != rx_data ) )
		{
			xSemaphoreTake( rx_semaphore, portMAX_DELAY );
			UART_TransferReceiveNonBlocking ( rx_config->uart_to_comm,
					rx_config->uart_handle_to_comm, &uart_rx_xfer, NULL );
			xSemaphoreTake( rx_semaphore, portMAX_DELAY );
			xSemaphoreGive( rx_semaphore );
			rx_data_ptr = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			rx_data_ptr->data = rx_data;
			rx_data_ptr->uart_handle_to_comm = &uart_pc_handle;
			rx_data_ptr->uart_to_comm = UART0;
			if ( ( RETURN_CHAR != rx_data ) && ( ESC_CHAR != rx_data ))
			{
				xQueueSend( rx_queue, &rx_data_ptr, portMAX_DELAY );
			}
		}
		vPortFree ( rx_config );
		if (ESC_CHAR == rx_data)
		{
			rx_data = 0;
			xEventGroupSetBits ( menu_event_handle, ESC_RECEIVED );
		}
		rx_data = uxQueueMessagesWaiting ( rx_queue );
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

	port_pin_config_t config_i2c =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt2,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig ( PORTB, 16, &config_uart );
	PORT_SetPinConfig ( PORTB, 17, &config_uart );
	PORT_SetPinConfig ( PORTB, 2, &config_i2c );
	PORT_SetPinConfig ( PORTB, 3, &config_i2c );
//UART configuration
	UART_GetDefaultConfig ( &uart_config );
	uart_config.baudRate_Bps = 115200U;
	uart_config.enableRx = pdTRUE;
	uart_config.enableTx = pdTRUE;
	UART_Init ( UART0, &uart_config, CLOCK_GetFreq ( UART0_CLK_SRC ) );
	UART_TransferCreateHandle ( UART0, &uart_pc_handle, UART0_UserCallback,
	NULL );

	i2c_master_config_t masterConfig;
	I2C_MasterGetDefaultConfig ( &masterConfig );
	masterConfig.baudRate_Bps = 100000;
	I2C_MasterInit ( I2C0, &masterConfig, CLOCK_GetFreq ( kCLOCK_BusClk ) );
	I2C_MasterTransferCreateHandle ( I2C0, &g_m_handle, i2c_master_callback,
	NULL );

	NVIC_EnableIRQ ( UART0_RX_TX_IRQn );
	NVIC_SetPriority ( UART0_RX_TX_IRQn, 8 );
	NVIC_EnableIRQ ( I2C0_IRQn );
	NVIC_SetPriority ( I2C0_IRQn, 7 );

	tx_semaphore = xSemaphoreCreateBinary();
	rx_semaphore = xSemaphoreCreateBinary();
	i2c_semaphore = xSemaphoreCreateBinary();
	i2c_mutex = xSemaphoreCreateMutex();

	uart_event_handle = xEventGroupCreate ();
	menu_event_handle = xEventGroupCreate ();
	i2c_event_handle = xEventGroupCreate ();

	rx_cfg_queue = xQueueCreate( 1, sizeof(void*) );
	addr_queue = xQueueCreate( 1, sizeof(uint16_t) );
	bcd_queue = xQueueCreate( 1, sizeof(uint8_t) );
	rx_queue = xQueueCreate( UART_BUFFER_SIZE, sizeof(void*) );
	tx_queue = xQueueCreate( UART_BUFFER_SIZE, sizeof(void*) );
	i2c_queue = xQueueCreate( MAX_I2C_PAGE_SIZE, sizeof(void*) );

//Task startup

	xTaskCreate ( esc_sequence_task, "ESCTask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES, NULL );
	xTaskCreate ( menu_sequence_task, "MenuTask", configMINIMAL_STACK_SIZE,
	NULL, configMAX_PRIORITIES - 1, NULL );

	xTaskCreate ( write_sequence_task, "WRtask", configMINIMAL_STACK_SIZE,
	NULL,
	configMAX_PRIORITIES - 2, NULL );
	xTaskCreate ( read_sequence_task, "RDtask", configMINIMAL_STACK_SIZE,
	NULL,
	configMAX_PRIORITIES - 2, NULL );

	xTaskCreate ( rx_task, "RXtask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 3, NULL );
	xTaskCreate ( tx_task, "TXtask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 3, NULL );

	xTaskCreate ( i2c_task, "I2CTask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 4, NULL );

	xTaskCreate ( addr_parser_task, "ADDRtask", configMINIMAL_STACK_SIZE,
	NULL,
	configMAX_PRIORITIES - 4, NULL );
	xTaskCreate ( bcd_parser_task, "BCDTask", configMINIMAL_STACK_SIZE,
	NULL,
	configMAX_PRIORITIES - 4, NULL );

	vTaskStartScheduler ();

	while ( 1 )
	{

	}
	return 0;
}
