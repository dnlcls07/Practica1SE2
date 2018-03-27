/*
 * menu_tasks.h
 *
 *  Created on: Mar 26, 2018
 *      Author: dceli
 */

#ifndef MENU_TASKS_H_
#define MENU_TASKS_H_

#include "FreeRTOS.h"

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_uart.h"
#include "fsl_i2c.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "uart_tasks.h"
#include "i2c_tasks.h"

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

/********************************************************************************************/
/*!
 	 \brief
 	 	 Starts up the tasks needed for the PC interface
 	 \param[in] void
 	 \return void
 */
void UART0_menu_init_task ( void * arg );

/********************************************************************************************/
/*!
 	 \brief
 	 	 Starts up the tasks needed for the Bluetooth interface
 	 \param[in] void
 	 \return void
 */
void UART1_menu_init_task ( void * arg );

/********************************************************************************************/
/*!
 	 \brief
 	 	 Converts the input data into a parsed hexadecimal address.
 	 \param[in] void
 	 \return void
 */
void addr_parser_task ( void * arg );

/********************************************************************************************/
/*!
 	 \brief
 	 	 Converts the input data into a parsed decimal data length.
 	 \param[in] void
 	 \return void
 */
void bcd_parser_task ( void * arg );

/********************************************************************************************/
/*!
 	 \brief
 	 	 High-priority task that is called when the user inputs the escape character.
 	 	 It recalls the menu as if the task was completed.
 	 \param[in] void
 	 \return void
 */
void esc_sequence_task ( void * arg );

/********************************************************************************************/
/*!
 	 \brief
 	 	 Menu sequence. It sends the menu options to the terminal and receives the user choice.
 	 	 It calls the corresponding sequence to execute. Sequences return here after finished.
 	 \param[in] void
 	 \return void
 */
void menu_sequence_task ( void * arg );

/********************************************************************************************/
/*!
 	 \brief
 	 	 Read memory sequence. Allows the user to write an address and a data length to retreive
 	 	 from the external memory.
 	 \param[in] void
 	 \return void
 */
void read_sequence_task ( void * arg );

/********************************************************************************************/
/*!
 	 \brief
 	 	 Write memory sequence. Allows the user th write an address and the input data to send
 	 	 to the external memory. Maximum capacity of 64 characters per write.
 	 \param[in] void
 	 \return void
 */
void write_sequence_task ( void * arg );

#endif /* MENU_TASKS_H_ */
