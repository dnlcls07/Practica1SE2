/*
 * uart_tasks.h
 *
 *  Created on: Mar 26, 2018
 *      Author: dceli
 */

#ifndef UART_TASKS_H_
#define UART_TASKS_H_

#include "menu_tasks.h"
#include "FreeRTOS.h"

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_uart.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#define RX_ENABLE (1<<0)
#define RX_DONE (1<<1)
#define TX_ENABLE (1<<2)
#define TX_DONE (1<<3)

//UART protection semaphores
SemaphoreHandle_t UART0_rx_semaphore;
SemaphoreHandle_t UART0_tx_semaphore;

SemaphoreHandle_t UART1_rx_semaphore;
SemaphoreHandle_t UART1_tx_semaphore;

uart_handle_t uart_pc_handle;
uart_handle_t uart_bt_handle;
uart_config_t uart_pc_config;
uart_config_t uart_bt_config;

/********************************************************************************************/
/*!
 	 \brief
 	 	 Starts up the UART0 (PC) and its tasks.
 	 \param[in] void
 	 \return void
 */
void UART0_init_task ( void * arg );

/********************************************************************************************/
/*!
 	 \brief
 	 	 Starts up the UART1 (Bluetooth) and its tasks.
 	 \param[in] void
 	 \return void
 */
void UART1_init_task ( void * arg );

/********************************************************************************************/
/*!
 	 \brief
 	 	 UART0 interruption callback. Clears the corresponding semaphores.
 	 \param[in] void
 	 \return void
 */
void UART0_UserCallback ( UART_Type *base, uart_handle_t *handle,
		status_t status, void *userData );

/********************************************************************************************/
/*!
 	 \brief
 	 	 UART1 interruption callback. Clears the corresponding semaphores.
 	 \param[in] void
 	 \return void
 */
void UART1_UserCallback ( UART_Type *base, uart_handle_t *handle,
		status_t status, void *userData );

/********************************************************************************************/
/*!
 	 \brief
 	 	 Transmission task. Sends out the received information on its queue to the corresponding
 	 	 terminal.
 	 \param[in] void
 	 \return void
 */
void tx_task ( void * arg );

/********************************************************************************************/
/*!
 	 \brief
 	 	 Reception task. Takes in the received information onto its queue until it receives a
 	 	 return character or a escape call.
 	 \param[in] void
 	 \return void
 */
void rx_task ( void * arg );

#endif /* UART_TASKS_H_ */
