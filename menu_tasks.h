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

#define UART0_INIT_DONE (1<<0)
#define UART1_INIT_DONE (1<<1)

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
