/*
 * menu_tasks.h
 *
 *  Created on: Mar 26, 2018
 *      Author: Daniel Celis & Gustavo Araiza
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

#define UART0_CHAT_RDY (1<<0)
#define UART1_CHAT_RDY (1<<1)

#define SET_SECONDS_DEFAULT 0x00
#define SET_MINUTES_DEFAULT 0x59
#define SET_HOURS_DEFAULT 0x23
#define SET_ADDRESS_DEFAULT 0x50
#define YEAR_DEFAULT 0x2018



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

/********************************************************************************************/
/*!
 \brief
 Chat sequence. It allows the communication between two terminals. The task actually just sends
 the user message to the other terminal.
 \param[in] void
 \return void
 */
void chat_sequence_task ( void * arg );

/********************************************************************************************/
/*!
 \brief
 Set Hour sequence. Esta funcion permite establecer la hora en el RTC a traves de una terminal
 \param[in] void
 \return void
 */
void sethour_sequence_task ( void * arg );

/********************************************************************************************/
/*!
 \brief
 Set date sequence. Esta funcion permite establecer la fecha en el RTC a traves de una terminal
 \param[in] void
 \return void
 */
void setdate_sequence_task ( void * arg );

/********************************************************************************************/
/*!
 \brief
 format sequence. Con esta funcion se puede cambiar el formato entre 12 y 24 horas.
 \param[in] void
 \return void
 */
void format_sequence_task ( void * arg );

/********************************************************************************************/
/*!
 \brief
 Read hour sequence. Con esta tarea se puede leer la hora en tiempo real para desplegar en
 una terminal
 \param[in] void
 \return void
 */
void readhour_sequence_task ( void * arg );

/********************************************************************************************/
/*!
 \brief
 Read date sequence. con esta tarea es posible leer la fecha del RTC y desplegarlo en una
 terminal.
 \param[in] void
 \return void
 */
void readdate_sequence_task ( void * arg );

/********************************************************************************************/
/*!
 \brief
 Eco sequence. esta tarea activa la funcion de modo eco en la LCD.
 \param[in] void
 \return void
 */
void eco_sequence_task ( void * arg );

/********************************************************************************************/
/*!
 \brief
 esta tarea establece por default unos valores predeterminados para e RTC y su funconamiento.
 \param[in] void
 \return void
 */
void GetTime_default_t(void * arg);


#endif /* MENU_TASKS_H_ */
