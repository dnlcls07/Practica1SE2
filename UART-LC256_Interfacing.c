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

#define RING_BUFFER_SIZE 1
#define RX_NEEDED (1<<0)
#define RX_nGOING (1<<1)
#define TX_ECHO_EN (1<<2)
#define TX_FULL (1<<3)

#define I2C_READ_EN (1<<0)
#define I2C_WRITE_EN (1<<1)

typedef struct
{
	UART_Type * uart_to_comm;
	uint8_t * string_ptr;
} uart_pkg_struct_t;

SemaphoreHandle_t i2c_semaphore;
i2c_master_handle_t g_m_handle;

EventGroupHandle_t uart_event_handle;
EventGroupHandle_t i2c_event_handle;

QueueHandle_t tx_queue;
QueueHandle_t rx_queue;
QueueHandle_t i2c_w_queue;
QueueHandle_t i2c_r_queue;
uart_handle_t uart_pc_handle;
uart_config_t uart_config;

size_t receivedBytes;

uint8_t rx_data;
uint8_t tx_data;

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
		xEventGroupSetBitsFromISR ( uart_event_handle, RX_nGOING,
				&xHigherPriorityTaskWoken );
	}
//	if (kStatus_UART_TxIdle == status)
//	{
//		xEventGroupSetBitsFromISR ( uart_event_handle, RX_EMPTY,
//				&xHigherPriorityTaskWoken );
//	}
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
		if (handle->transfer.direction == kI2C_Write)
		{
			xEventGroupClearBitsFromISR ( i2c_event_handle, I2C_WRITE_EN );
		}
		else if (handle->transfer.direction == kI2C_Read)
		{
			xEventGroupClearBitsFromISR ( i2c_event_handle, I2C_READ_EN );
		}
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void i2c_write_task ( void * arg )
{
	i2c_master_transfer_t i2c_master_xfer;
	uint8_t data_buffer_out;
	for ( ;; )
	{
		xEventGroupWaitBits ( i2c_event_handle, I2C_WRITE_EN, pdTRUE, pdTRUE,
		portMAX_DELAY );
		xQueueReceive( rx_queue, &data_buffer_out, portMAX_DELAY );
		i2c_master_xfer.slaveAddress = 0x50;
		i2c_master_xfer.direction = kI2C_Write;
		i2c_master_xfer.subaddress = 0x05A0;
		i2c_master_xfer.subaddressSize = 2;
		i2c_master_xfer.flags = kI2C_TransferDefaultFlag;
		i2c_master_xfer.data = &data_buffer_out;
		i2c_master_xfer.dataSize = sizeof ( data_buffer_out );
		I2C_MasterTransferNonBlocking ( I2C0, &g_m_handle, &i2c_master_xfer );
		xSemaphoreTake( i2c_semaphore, portMAX_DELAY );
		xSemaphoreGive( i2c_semaphore );
		xEventGroupSetBits ( i2c_event_handle, I2C_READ_EN );
	}
}

void i2c_read_task ( void * arg )
{
	i2c_master_transfer_t i2c_master_xfer;
	uint8_t data_buffer;
	for ( ;; )
	{
		xEventGroupWaitBits ( i2c_event_handle, I2C_READ_EN, pdTRUE, pdTRUE,
		portMAX_DELAY );
		xSemaphoreGive( i2c_semaphore );
		vTaskDelay ( pdMS_TO_TICKS( 100 ) );
		i2c_master_xfer.slaveAddress = 0x50;
		i2c_master_xfer.direction = kI2C_Read;
		i2c_master_xfer.subaddress = 0x05A0;
		i2c_master_xfer.subaddressSize = 2;
		i2c_master_xfer.flags = kI2C_TransferDefaultFlag;
		i2c_master_xfer.data = &data_buffer;
		i2c_master_xfer.dataSize = sizeof ( data_buffer );
		xSemaphoreTake( i2c_semaphore, portMAX_DELAY );
		I2C_MasterTransferNonBlocking ( I2C0, &g_m_handle, &i2c_master_xfer );
		xSemaphoreTake( i2c_semaphore, portMAX_DELAY );
		xSemaphoreGive( i2c_semaphore );
		xQueueSend(tx_queue,&data_buffer,portMAX_DELAY);
		xEventGroupSetBits ( uart_event_handle, TX_ECHO_EN );
	}
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
		xQueueReceive( tx_queue, &tx_data, portMAX_DELAY );
		UART_TransferSendNonBlocking ( UART0, &uart_pc_handle,
				&uart_echo_xfer );
		xEventGroupSetBits ( uart_event_handle, RX_NEEDED );
		xEventGroupClearBits ( uart_event_handle, TX_ECHO_EN );
	}
}

void rx_task ( void * arg )
{
	uint8_t rx_data;
	uart_transfer_t uart_rx_xfer;
	uart_rx_xfer.data = &rx_data;
	uart_rx_xfer.dataSize = sizeof(char);
	xEventGroupSetBits ( uart_event_handle, RX_nGOING | RX_NEEDED );
	for ( ;; )
	{
		xEventGroupWaitBits ( uart_event_handle, RX_NEEDED | RX_nGOING, pdFALSE,
		pdTRUE,
		portMAX_DELAY );
		UART_TransferReceiveNonBlocking ( UART0, &uart_pc_handle, &uart_rx_xfer,
		NULL );
		xEventGroupClearBits ( uart_event_handle, RX_nGOING );
		xEventGroupWaitBits ( uart_event_handle, RX_nGOING, pdFALSE, pdTRUE,
		portMAX_DELAY );
		if (RETURN_CHAR != rx_data)
			xQueueSend( rx_queue, &rx_data, portMAX_DELAY );
		else
		{
			xEventGroupClearBits ( uart_event_handle, RX_NEEDED );
			xEventGroupSetBits ( i2c_event_handle, I2C_WRITE_EN );
		}
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
	CLOCK_EnableClock ( kCLOCK_I2c0 );

	port_pin_config_t config_uart =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt3,
			kPORT_UnlockRegister, };

	port_pin_config_t config_i2c =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt2,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig ( PORTB, 2, &config_i2c );
	PORT_SetPinConfig ( PORTB, 3, &config_i2c );

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

	i2c_semaphore = xSemaphoreCreateBinary ();
	i2c_event_handle = xEventGroupCreate ();
	uart_event_handle = xEventGroupCreate ();

	rx_queue = xQueueCreate( 1, sizeof(char) );
	tx_queue = xQueueCreate( 1, sizeof(char) );

	//Task startup

	xTaskCreate ( rx_task, "RXtask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 4, NULL );
	xTaskCreate ( tx_task, "TXtask", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 1, NULL );
	xTaskCreate ( i2c_write_task, "I2C_MEM_W_Task", configMINIMAL_STACK_SIZE,
	NULL,
	configMAX_PRIORITIES - 3, NULL );
	xTaskCreate ( i2c_read_task, "I2C_MEM_R_TASK", configMINIMAL_STACK_SIZE,
	NULL, configMAX_PRIORITIES - 2, NULL );
	vTaskStartScheduler ();

	while ( 1 )
	{

	}
	return 0;
}

