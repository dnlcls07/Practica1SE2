/*
 * definitions.h
 *
 *  Created on: Mar 27, 2018
 *      Author: dceli
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include "FreeRTOS.h"

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_uart.h"
#include "fsl_i2c.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#define ADDR_ENABLE (1<<0)
#define ADDR_DONE (1<<1)
#define BCD_ENABLE (1<<2)
#define BCD_DONE (1<<3)

#define READ_SEQ_ENABLE (1<<4)
#define READ_SEQ_DONE (1<<5)
#define WRITE_SEQ_ENABLE (1<<6)
#define WRITE_SEQ_DONE (1<<7)
#define CHAT_SEQ_ENABLE (1<<8)
#define CHAT_SEQ_DONE (1<<9)
#define ESC_RECEIVED (1<<10)
#define ESC_B2MENU (1<<11)

#define RETURN_CHAR 0x0D
#define ESC_CHAR 0x23
#define ALL_EVENTS 0x7FFFFF

#define UART_BUFFER_SIZE 64
#define MAX_I2C_PAGE_SIZE 64
#define MEM_ADDR_SIZE 4

typedef struct
{
	UART_Type * uart_calling;
	uart_handle_t * uart_handle;
	EventGroupHandle_t uart_event_handle;
	EventGroupHandle_t menu_event_handle;
	EventGroupHandle_t i2c_event_handle;
	EventGroupHandle_t CHAT_uart_handle;
	QueueHandle_t tx_queue;
	QueueHandle_t rx_queue;
	QueueHandle_t rx_cfg_queue;
	QueueHandle_t addr_queue;
	QueueHandle_t bcd_queue;
	QueueHandle_t i2c_queue;
} menu_cfg_struct_t;

typedef struct
{
	UART_Type * uart_to_comm;
	uart_handle_t * uart_handle_to_comm;
	uint8_t data;
} uart_pkg_struct_t;

#endif /* DEFINITIONS_H_ */
