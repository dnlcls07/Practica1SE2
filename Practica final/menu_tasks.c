/*
 * menu_tasks.c
 *
 *  Created on: Mar 26, 2018
 *      Author: Daniel Celis & Gustavo Araiza
 */

#include "menu_tasks.h"

menu_cfg_struct_t * UART0_menu;
menu_cfg_struct_t * UART1_menu;

EventGroupHandle_t init_event;
EventGroupHandle_t chat_event;

TaskHandle_t UART0_read_seq_handle;
TaskHandle_t UART0_write_seq_handle;
TaskHandle_t UART0_chat_seq_handle;
TaskHandle_t UART0_sethour_seq_handle;
TaskHandle_t UART0_setdate_seq_handle;
TaskHandle_t UART0_format_seq_handle;
TaskHandle_t UART0_readhour_seq_handle;
TaskHandle_t UART0_readdate_seq_handle;
TaskHandle_t UART0_eco_seq_handle;

TaskHandle_t UART1_read_seq_handle;
TaskHandle_t UART1_write_seq_handle;
TaskHandle_t UART1_chat_seq_handle;
TaskHandle_t UART1_sethour_seq_handle;
TaskHandle_t UART1_setdate_seq_handle;
TaskHandle_t UART1_format_seq_handle;
TaskHandle_t UART1_readhour_seq_handle;
TaskHandle_t UART1_readdate_seq_handle;
TaskHandle_t UART1_eco_seq_handle;
uint8_t Flag12_24hour = 0;

void UART0_menu_init_task ( void * arg )
{
	UART0_menu = pvPortMalloc ( sizeof(menu_cfg_struct_t) );
	UART0_menu->addr_queue = xQueueCreate( 1, sizeof(uint16_t) );
	UART0_menu->bcd_queue = xQueueCreate( 1, sizeof(uint16_t) );
	UART0_menu->i2c_event_handle = xEventGroupCreate ();
	UART0_menu->menu_event_handle = xEventGroupCreate ();
	UART0_menu->uart_event_handle = xEventGroupCreate ();
	UART0_menu->uart_handle = UART0_get_handle ();
	UART0_menu->CHAT_uart_handle = UART1_get_handle ();
	UART0_menu->uart_calling = UART0;
	UART0_menu->rx_queue = xQueueCreate( UART_BUFFER_SIZE, sizeof(void*) );
	UART0_menu->tx_queue = xQueueCreate( UART_BUFFER_SIZE, sizeof(void*) );
	UART0_menu->i2c_queue = xQueueCreate( MAX_I2C_PAGE_SIZE, sizeof(void*) );
	UART0_menu->rx_cfg_queue = xQueueCreate( 1, sizeof(void*) );

	init_event = xEventGroupCreate ();
	chat_event = xEventGroupCreate ();

	xTaskCreate ( i2c_task, "UART0_I2CTask", configMINIMAL_STACK_SIZE,
			( void * ) UART0_menu, configMAX_PRIORITIES - 3, NULL );
	xTaskCreate ( esc_sequence_task, "UART0_ESCTask", configMINIMAL_STACK_SIZE,
			( void * ) UART0_menu, configMAX_PRIORITIES, NULL );
	////////////////////////////////////////////////////////////////////////////
	xTaskCreate ( write_sequence_task, "UART0_WRtask", configMINIMAL_STACK_SIZE, //write task
			( void * ) UART0_menu, configMAX_PRIORITIES - 2,
			&UART0_write_seq_handle );
	xTaskCreate ( read_sequence_task, "UART0_RDtask", configMINIMAL_STACK_SIZE, //read task
			( void * ) UART0_menu, configMAX_PRIORITIES - 2,
			&UART0_read_seq_handle );
	xTaskCreate ( chat_sequence_task, "UART0_CHATtask",configMINIMAL_STACK_SIZE * 2, //chat task
			( void * ) UART0_menu,configMAX_PRIORITIES - 2,
			&UART0_chat_seq_handle );
	xTaskCreate ( sethour_sequence_task, "UART0_STHtask",configMINIMAL_STACK_SIZE, //set hour task
			( void * ) UART0_menu,configMAX_PRIORITIES - 2,
			&UART0_sethour_seq_handle );
	xTaskCreate ( setdate_sequence_task, "UART0_STDtask",configMINIMAL_STACK_SIZE, //set date task
			( void * ) UART0_menu,configMAX_PRIORITIES - 2,
			&UART0_setdate_seq_handle );
	xTaskCreate ( format_sequence_task, "UART0_FORMATtask",configMINIMAL_STACK_SIZE, //Format task
			( void * ) UART0_menu,configMAX_PRIORITIES - 2,
			&UART0_format_seq_handle );
	xTaskCreate ( readhour_sequence_task, "UART0_RDHtask",configMINIMAL_STACK_SIZE, //read hour task
			( void * ) UART0_menu,configMAX_PRIORITIES - 2,
			&UART0_readhour_seq_handle );
	xTaskCreate ( readdate_sequence_task, "UART0_RDDtask",configMINIMAL_STACK_SIZE, //read date task
			( void * ) UART0_menu,configMAX_PRIORITIES - 2,
			&UART0_readdate_seq_handle );
	xTaskCreate ( eco_sequence_task, "UART0_ECOtask",configMINIMAL_STACK_SIZE, //eco lcd task
			( void * ) UART0_menu,configMAX_PRIORITIES - 2,
			&UART0_eco_seq_handle );
	////////////////////////////////////////////////////////////////////////////
	xTaskCreate ( UART0_init_task, "UART0_init", configMINIMAL_STACK_SIZE,
			( void * ) UART0_menu, configMAX_PRIORITIES, NULL );
	xTaskCreate ( i2c_init_task, "UART0_I2C_Init", configMINIMAL_STACK_SIZE,
			( void * ) UART0_menu, configMAX_PRIORITIES, NULL );

	xTaskCreate ( addr_parser_task, "UART0_ADDRtask", configMINIMAL_STACK_SIZE,
			( void * ) UART0_menu, configMAX_PRIORITIES - 4, NULL );
	xTaskCreate ( bcd_parser_task, "UART0_BCDTask", configMINIMAL_STACK_SIZE,
			( void * ) UART0_menu, configMAX_PRIORITIES - 4, NULL );
	xTaskCreate ( menu_sequence_task, "UART0_MenuTask",
			configMINIMAL_STACK_SIZE, ( void * ) UART0_menu,
			configMAX_PRIORITIES - 1, NULL );
	xEventGroupSetBits ( init_event, UART0_INIT_DONE );
	vTaskDelete ( NULL );
}

void UART1_menu_init_task ( void * arg )
{
	UART1_menu = pvPortMalloc ( sizeof(menu_cfg_struct_t) );

	UART1_menu->addr_queue = xQueueCreate( 1, sizeof(uint16_t) );
	UART1_menu->bcd_queue = xQueueCreate( 1, sizeof(uint16_t) );
	UART1_menu->i2c_event_handle = xEventGroupCreate ();
	UART1_menu->menu_event_handle = xEventGroupCreate ();
	UART1_menu->uart_event_handle = xEventGroupCreate ();
	UART1_menu->uart_handle = UART1_get_handle ();
	UART1_menu->CHAT_uart_handle = UART0_get_handle ();
	UART1_menu->uart_calling = UART1;
	UART1_menu->rx_queue = xQueueCreate( UART_BUFFER_SIZE, sizeof(void*) );
	UART1_menu->tx_queue = xQueueCreate( UART_BUFFER_SIZE, sizeof(void*) );
	UART1_menu->i2c_queue = xQueueCreate( MAX_I2C_PAGE_SIZE, sizeof(void*) );
	UART1_menu->rx_cfg_queue = xQueueCreate( 1, sizeof(void*) );

	xTaskCreate ( esc_sequence_task, "UART1_ESCTask", configMINIMAL_STACK_SIZE,
			( void * ) UART1_menu, configMAX_PRIORITIES - 1, NULL );
	xTaskCreate ( menu_sequence_task, "UART1_MenuTask",
			configMINIMAL_STACK_SIZE, ( void * ) UART1_menu,
			configMAX_PRIORITIES - 1, NULL );
	xTaskCreate ( i2c_task, "UART1_I2CTask", configMINIMAL_STACK_SIZE,
			( void * ) UART1_menu, configMAX_PRIORITIES - 3, NULL );
	//////////////////////////////////////////////////////////////////////////////
	xTaskCreate ( write_sequence_task, "UART1_WRtask", configMINIMAL_STACK_SIZE,
			( void * ) UART1_menu, configMAX_PRIORITIES - 2,
			&UART1_write_seq_handle );
	xTaskCreate ( read_sequence_task, "UART1_RDtask", configMINIMAL_STACK_SIZE,
			( void * ) UART1_menu, configMAX_PRIORITIES - 2,
			&UART1_read_seq_handle );
	xTaskCreate ( chat_sequence_task, "UART1_CHATtask", configMINIMAL_STACK_SIZE,
			( void * ) UART1_menu, configMAX_PRIORITIES - 2,
			&UART1_chat_seq_handle );
	xTaskCreate ( sethour_sequence_task, "UART1_STHtask",configMINIMAL_STACK_SIZE, //set hour task
			( void * ) UART1_menu,configMAX_PRIORITIES - 2,
			&UART1_sethour_seq_handle );
	xTaskCreate ( setdate_sequence_task, "UART1_STDtask",configMINIMAL_STACK_SIZE, //set date task
			( void * ) UART1_menu,configMAX_PRIORITIES - 2,
			&UART1_setdate_seq_handle );
	xTaskCreate ( format_sequence_task, "UART1_FORMATtask",configMINIMAL_STACK_SIZE, //Format task
			( void * ) UART1_menu,configMAX_PRIORITIES - 2,
			&UART1_format_seq_handle );
	xTaskCreate ( readhour_sequence_task, "UART1_RDHtask",configMINIMAL_STACK_SIZE, //read hour task
			( void * ) UART1_menu,configMAX_PRIORITIES - 2,
			&UART1_readhour_seq_handle );
	xTaskCreate ( readdate_sequence_task, "UART1_RDDtask",configMINIMAL_STACK_SIZE, //read date task
			( void * ) UART1_menu,configMAX_PRIORITIES - 2,
			&UART1_readdate_seq_handle );
	xTaskCreate ( eco_sequence_task, "UART1_ECOtask",configMINIMAL_STACK_SIZE, //eco lcd task
			( void * ) UART1_menu,configMAX_PRIORITIES - 2,
			&UART1_eco_seq_handle );
	///////////////////////////////////////////////////////////////////////////////
	xTaskCreate ( UART1_init_task, "UART1_UART_init", configMINIMAL_STACK_SIZE,
			( void * ) UART1_menu, configMAX_PRIORITIES, NULL );

	xTaskCreate ( addr_parser_task, "UART1_ADDRtask", configMINIMAL_STACK_SIZE,
			( void * ) UART1_menu, configMAX_PRIORITIES - 4, NULL );
	xTaskCreate ( bcd_parser_task, "UART1_BCDTask", configMINIMAL_STACK_SIZE,
			( void * ) UART1_menu, configMAX_PRIORITIES - 4, NULL );
	xEventGroupSetBits ( init_event, UART1_INIT_DONE );
	vTaskDelete ( NULL );
}

void addr_parser_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uart_pkg_struct_t * uart_pkg;
	uint8_t msg_cnt;
	uint8_t addr [ 4 ];
	uint16_t relative_addr;
	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, ADDR_ENABLE,
				pdTRUE, pdTRUE, portMAX_DELAY );
		for ( msg_cnt = 0; msg_cnt <= 3; msg_cnt++ )
		{
			xQueueReceive( cfg_struct->rx_queue, &uart_pkg, portMAX_DELAY );
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
		xQueueSend( cfg_struct->addr_queue, &relative_addr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->menu_event_handle, ADDR_DONE );
	}
}

void bcd_parser_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uint8_t bytesReceived;
	uint8_t bcd_data [ 2 ];
	uart_pkg_struct_t * uart_pkg;
	uint16_t parsed_data;
	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, BCD_ENABLE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		for ( bytesReceived = 0;
				uxQueueMessagesWaiting ( cfg_struct->rx_queue ) != pdFALSE;
				bytesReceived++ )
		{
			xQueueReceive( cfg_struct->rx_queue, &uart_pkg, portMAX_DELAY );
			bcd_data [ bytesReceived ] = uart_pkg->data;
			vPortFree ( uart_pkg );
		}
		parsed_data = ( ( bcd_data [ 0 ] - '0' ) * 10 )
																																																																												+ ( bcd_data [ 1 ] - '0' );
		xQueueSend( cfg_struct->bcd_queue, &parsed_data, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->menu_event_handle, BCD_DONE );
	}
}

void esc_sequence_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, ESC_RECEIVED,
				pdTRUE, pdTRUE, portMAX_DELAY );
		xQueueReset( cfg_struct->tx_queue );
		xQueueReset( cfg_struct->rx_queue );
		if (UART0 == cfg_struct->uart_calling)
		{
			xEventGroupClearBits ( chat_event, UART0_CHAT_RDY );
			vTaskDelete ( UART0_read_seq_handle );
			vTaskDelete ( UART0_write_seq_handle );
			vTaskDelete ( UART0_chat_seq_handle );
			vTaskDelete ( UART0_sethour_seq_handle );
			vTaskDelete ( UART0_setdate_seq_handle );
			vTaskDelete ( UART0_format_seq_handle );
			vTaskDelete ( UART0_readhour_seq_handle );
			vTaskDelete ( UART0_readdate_seq_handle );
			vTaskDelete ( UART0_eco_seq_handle );

			xTaskCreate ( write_sequence_task, "UART0_WRtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART0_write_seq_handle );
			xTaskCreate ( read_sequence_task, "UART0_RDtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART0_read_seq_handle );
			xTaskCreate ( chat_sequence_task, "UART0_CHATtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART0_chat_seq_handle );
			xTaskCreate ( sethour_sequence_task, "UART0_STHtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART0_sethour_seq_handle );
			xTaskCreate ( setdate_sequence_task, "UART0_STDtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART0_setdate_seq_handle );
			xTaskCreate ( format_sequence_task, "UART0_FORMATtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART0_format_seq_handle );
			xTaskCreate ( readhour_sequence_task, "UART0_RDHtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART0_readhour_seq_handle );
			xTaskCreate ( readdate_sequence_task, "UART0_RDDtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART0_readdate_seq_handle );
			xTaskCreate ( eco_sequence_task, "UART0_ECOtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART0_eco_seq_handle );
		}
		else
		{
			xEventGroupClearBits ( chat_event, UART1_CHAT_RDY );
			vTaskDelete ( UART1_read_seq_handle );
			vTaskDelete ( UART1_write_seq_handle );
			vTaskDelete ( UART1_chat_seq_handle );
			vTaskDelete ( UART1_sethour_seq_handle );
			vTaskDelete ( UART1_setdate_seq_handle );
			vTaskDelete ( UART1_format_seq_handle );
			vTaskDelete ( UART1_readhour_seq_handle );
			vTaskDelete ( UART1_readdate_seq_handle );
			vTaskDelete ( UART1_eco_seq_handle );

			xTaskCreate ( write_sequence_task, "UART1_WRtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART1_write_seq_handle );
			xTaskCreate ( read_sequence_task, "UART1_RDtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART1_read_seq_handle );
			xTaskCreate ( chat_sequence_task, "UART1_CHATtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART1_chat_seq_handle );
			xTaskCreate ( sethour_sequence_task, "UART1_STHtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART1_sethour_seq_handle );
			xTaskCreate ( setdate_sequence_task, "UART1_STDtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART1_setdate_seq_handle );
			xTaskCreate ( format_sequence_task, "UART1_FORMATtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART1_format_seq_handle );
			xTaskCreate ( readhour_sequence_task, "UART1_RDHtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART1_readhour_seq_handle );
			xTaskCreate ( readdate_sequence_task, "UART1_RDDtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART1_readdate_seq_handle );
			xTaskCreate ( eco_sequence_task, "UART1_ECOtask",
					configMINIMAL_STACK_SIZE, ( void * ) cfg_struct,
					configMAX_PRIORITIES - 2, &UART1_eco_seq_handle );
		}
		xEventGroupSetBits ( cfg_struct->menu_event_handle, ESC_B2MENU );
	}
}

void menu_sequence_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uint8_t menu_msg [] =
			"\n\rChoose a menu:\n\r1) Write\n\r2) Read\n\r3) Chat";
	uint8_t menu_msg1 [] =
			"\n\r4) Set hour\n\r5) Set date\n\r6) Format hour";
	uint8_t menu_msg2 [] =
			"\n\r7) Read hour\n\r8) Read date\n\r9) ECO lcd\n\r";
	uart_pkg_struct_t * uart_pkg;
	uint8_t msg_cnt;
	uint32_t waitForSeq;
	xEventGroupWaitBits ( init_event, UART0_INIT_DONE | UART1_INIT_DONE, pdTRUE,
			pdTRUE, portMAX_DELAY );
	for ( ;; )
	{
		for ( msg_cnt = 0; menu_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = menu_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		//////////////////////////////////////////////////////////////////////
		for ( msg_cnt = 0; menu_msg1 [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = menu_msg1 [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		//////////////////////////////////////////////////////////////////////
		for ( msg_cnt = 0; menu_msg2 [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = menu_msg2 [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		/////////////////////////////////////////////////////////////////////////
		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
		uart_pkg->uart_to_comm = cfg_struct->uart_calling;
		xQueueSend( cfg_struct->rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		xQueueReceive( cfg_struct->rx_queue, &uart_pkg, portMAX_DELAY );
		switch ( uart_pkg->data )
		{
		case '1' :
			xEventGroupSetBits ( cfg_struct->menu_event_handle,
					WRITE_SEQ_ENABLE );
			waitForSeq = WRITE_SEQ_DONE;
			break;
		case '2' :
			xEventGroupSetBits ( cfg_struct->menu_event_handle,
					READ_SEQ_ENABLE );
			waitForSeq = READ_SEQ_DONE;
			break;
		case '3' :
			xEventGroupSetBits ( cfg_struct->menu_event_handle,
					CHAT_SEQ_ENABLE );
			waitForSeq = CHAT_SEQ_DONE;
			break;
		case '4' :
			xEventGroupSetBits ( cfg_struct->menu_event_handle,
					SETHOUR_SEQ_ENABLE );
			waitForSeq = SETHOUR_SEQ_DONE;
			break;
		case '5' :
			xEventGroupSetBits ( cfg_struct->menu_event_handle,
					SETDATE_SEQ_ENABLE );
			waitForSeq = SETDATE_SEQ_DONE;
			break;
		case '6' :
			xEventGroupSetBits ( cfg_struct->menu_event_handle,
					FORMAT_SEQ_ENABLE );
			waitForSeq = FORMAT_SEQ_DONE;
			break;
		case '7' :
			xEventGroupSetBits ( cfg_struct->menu_event_handle,
					READHOUR_SEQ_ENABLE );
			waitForSeq = READHOUR_SEQ_DONE;
			break;
		case '8' :
			xEventGroupSetBits ( cfg_struct->menu_event_handle,
					READDATE_SEQ_ENABLE );
			waitForSeq = READDATE_SEQ_DONE;
			break;
		case '9' :
			xEventGroupSetBits ( cfg_struct->menu_event_handle,
					ECO_SEQ_ENABLE );
			waitForSeq = ECO_SEQ_DONE;
			break;
		default :
			break;
		}
		vPortFree ( uart_pkg );
		xEventGroupWaitBits ( cfg_struct->menu_event_handle,
				waitForSeq | ESC_B2MENU,
				pdTRUE, pdFALSE, portMAX_DELAY );
	}
}

void read_sequence_task ( void * arg )
{
	i2c_master_transfer_t * i2c_xfer_ptr;
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uint8_t msg_cnt;
	uint8_t read_msg [ ] = "\n\rRead address:\n\r\0";
	uint8_t size_msg [ ] = "\n\rRead size:\n\r\0";

	uint16_t addr;
	uint8_t data_size;

	uart_pkg_struct_t * uart_pkg;
	uart_pkg_struct_t * i2c_pkg;
	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, READ_SEQ_ENABLE,
				pdTRUE,pdTRUE, portMAX_DELAY );

		for ( msg_cnt = 0; read_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = read_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
		uart_pkg->uart_to_comm = cfg_struct->uart_calling;
		;
		xQueueSend( cfg_struct->rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->menu_event_handle, ADDR_ENABLE );
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, ADDR_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		for ( msg_cnt = 0; size_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = size_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
		uart_pkg->uart_to_comm = cfg_struct->uart_calling;
		xQueueSend( cfg_struct->rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->menu_event_handle, BCD_ENABLE );
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, BCD_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );

		xQueueReceive( cfg_struct->addr_queue, &addr, portMAX_DELAY );
		xQueueReceive( cfg_struct->bcd_queue, &data_size, portMAX_DELAY );
		uint8_t i2c_received_data [ data_size ];

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Read;
		i2c_xfer_ptr->subaddress = addr;
		i2c_xfer_ptr->subaddressSize = 2;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &i2c_received_data [ 0 ];
		i2c_xfer_ptr->dataSize = data_size;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		for ( msg_cnt = 0; msg_cnt < data_size; msg_cnt++ )
		{
			i2c_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			i2c_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			i2c_pkg->uart_to_comm = cfg_struct->uart_calling;
			i2c_pkg->data = i2c_received_data [ msg_cnt ];
			xQueueSend( cfg_struct->tx_queue, &i2c_pkg, portMAX_DELAY );
		}
		vPortFree ( i2c_xfer_ptr );
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE,
				portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->menu_event_handle, READ_SEQ_DONE );
	}
}

void write_sequence_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uint8_t msg_cnt;
	uint8_t read_msg [ ] = "\n\rWrite address:\n\r\0";
	uint8_t msg_msg [ ] = "\n\rWrite message:\n\r\0";
	uart_pkg_struct_t * uart_pkg;
	//	uint8_t msg_received [ UART_BUFFER_SIZE ];
	uint8_t msg_size;
	uint16_t addr;
	i2c_master_transfer_t * i2c_xfer_ptr;

	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, WRITE_SEQ_ENABLE,
				pdTRUE,pdTRUE, portMAX_DELAY );
		for ( msg_cnt = 0; read_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = read_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE, portMAX_DELAY );

		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
		uart_pkg->uart_to_comm = cfg_struct->uart_calling;
		xQueueSend( cfg_struct->rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_DONE, pdTRUE,
				pdTRUE,	portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->menu_event_handle, ADDR_ENABLE );
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, ADDR_DONE, pdTRUE,
				pdTRUE, portMAX_DELAY );

		for ( msg_cnt = 0; msg_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = msg_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE, portMAX_DELAY );

		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
		uart_pkg->uart_to_comm = cfg_struct->uart_calling;
		xQueueSend( cfg_struct->rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_DONE, pdTRUE,
				pdTRUE, portMAX_DELAY );

		xQueueReceive( cfg_struct->addr_queue, &addr, portMAX_DELAY );
		msg_size = uxQueueMessagesWaiting ( cfg_struct->rx_queue );

		uint8_t msg_received [ msg_size ];

		for ( msg_cnt = 0; msg_cnt < msg_size; msg_cnt++ )
		{
			xQueueReceive( cfg_struct->rx_queue, &uart_pkg, portMAX_DELAY );
			msg_received [ msg_cnt ] = uart_pkg->data;
			vPortFree ( uart_pkg );
		}

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Write;
		i2c_xfer_ptr->subaddress = addr;
		i2c_xfer_ptr->subaddressSize = 2;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &msg_received [ 0 ];
		i2c_xfer_ptr->dataSize = msg_size;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );

		xEventGroupSetBits ( cfg_struct->menu_event_handle, WRITE_SEQ_DONE );
	}
}

void chat_sequence_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uint8_t waiting_message [ ] = "\n\rWaiting for the other terminal\n\r";
	uint8_t start_message [ ] = "\n\rStarting communication\n\r";
	uint8_t answer_header [ ] = "\n\rOTHER: ";
	uint8_t answer_tail [ ] = "\n\r";
	uint8_t finish_message [ ] = "\n\rChat finished\n\rPlease press ESC\n\r";
	uint8_t msg_cnt;
	uint8_t msg_size;
	uint32_t chat_signal;
	uart_pkg_struct_t * uart_pkg;
	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, CHAT_SEQ_ENABLE,
				pdTRUE, pdTRUE, portMAX_DELAY );
		if (UART0 == cfg_struct->uart_calling)
		{
			xEventGroupSetBits ( chat_event, UART0_CHAT_RDY );
		}
		else
		{
			xEventGroupSetBits ( chat_event, UART1_CHAT_RDY );
		}
		chat_signal = xEventGroupWaitBits ( chat_event,
				UART0_CHAT_RDY | UART1_CHAT_RDY, pdFALSE, pdTRUE, 0 );
		if ( ( UART0_CHAT_RDY | UART1_CHAT_RDY ) != chat_signal)
		{
			for ( msg_cnt = 0; waiting_message [ msg_cnt ] != '\0'; msg_cnt++ )
			{
				uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
				uart_pkg->data = waiting_message [ msg_cnt ];
				uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
				uart_pkg->uart_to_comm = cfg_struct->uart_calling;
				xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
			}
			xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
			xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE,
					pdTRUE, pdTRUE, portMAX_DELAY );
			if (UART0 == cfg_struct->uart_calling)
			{
				xEventGroupWaitBits ( chat_event, UART1_CHAT_RDY, pdFALSE,
						pdTRUE, portMAX_DELAY );
			}
			else
			{
				xEventGroupWaitBits ( chat_event, UART0_CHAT_RDY, pdFALSE,
						pdTRUE, portMAX_DELAY );
			}
			chat_signal = xEventGroupWaitBits ( chat_event,
					UART0_CHAT_RDY | UART1_CHAT_RDY, pdFALSE, pdTRUE, 0 );
		}

		for ( msg_cnt = 0; start_message [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = start_message [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE,
				pdTRUE, pdTRUE, portMAX_DELAY );

		while ( ( UART0_CHAT_RDY | UART1_CHAT_RDY ) == chat_signal )
		{
			chat_signal = xEventGroupWaitBits ( chat_event,
					UART0_CHAT_RDY | UART1_CHAT_RDY, pdFALSE, pdTRUE, 0 );

			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = 0;
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->rx_cfg_queue, &uart_pkg, portMAX_DELAY );

			xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_ENABLE );
			xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_DONE,
					pdTRUE,
					pdTRUE, portMAX_DELAY );

			msg_size = uxQueueMessagesWaiting ( cfg_struct->rx_queue );
			uint8_t msg_to_send [ msg_size ];

			for ( msg_cnt = 0; answer_header [ msg_cnt ] != '\0'; msg_cnt++ )
			{
				uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
				uart_pkg->data = answer_header [ msg_cnt ];
				uart_pkg->uart_handle_to_comm = cfg_struct->CHAT_uart_handle;
				if (UART0 == cfg_struct->uart_calling)
				{
					uart_pkg->uart_to_comm = UART1;
				}
				else
				{
					uart_pkg->uart_to_comm = UART0;
				}
				xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
			}

			for ( msg_cnt = 0; msg_cnt < msg_size; msg_cnt++ )
			{
				xQueueReceive( cfg_struct->rx_queue, &uart_pkg, portMAX_DELAY );
				msg_to_send [ msg_cnt ] = uart_pkg->data;
				vPortFree ( uart_pkg );
			}

			for ( msg_cnt = 0; msg_cnt < msg_size; msg_cnt++ )
			{
				uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
				uart_pkg->data = msg_to_send [ msg_cnt ];
				uart_pkg->uart_handle_to_comm = cfg_struct->CHAT_uart_handle;
				if (UART0 == cfg_struct->uart_calling)
				{
					uart_pkg->uart_to_comm = UART1;
				}
				else
				{
					uart_pkg->uart_to_comm = UART0;
				}
				xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
			}
			for ( msg_cnt = 0; answer_tail [ msg_cnt ] != '\0'; msg_cnt++ )
			{
				uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
				uart_pkg->data = answer_tail [ msg_cnt ];
				uart_pkg->uart_handle_to_comm = cfg_struct->CHAT_uart_handle;
				if (UART0 == cfg_struct->uart_calling)
				{
					uart_pkg->uart_to_comm = UART1;
				}
				else
				{
					uart_pkg->uart_to_comm = UART0;
				}
				xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
			}

			xEventGroupSetBits ( cfg_struct->uart_event_handle,
					TX_ENABLE );
			xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE,
					pdTRUE, pdTRUE, portMAX_DELAY );
		}
		for ( msg_cnt = 0; finish_message [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = finish_message [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
	}
}

void sethour_sequence_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uint8_t msg_cnt;
	uint8_t count_msg = 0;
	uint8_t sethour_msg [ ] = "\n\rEscribir hora en formato hh:mm:ss\n\r\0";
	uint8_t finish_msg [ ] = "\n\rLa hora cambio exitosamente...\n\r\0";

	uart_pkg_struct_t * uart_pkg;
	uint8_t msg_size;
	uint8_t hora[2];
	uint8_t minutos[2];
	uint8_t segundos[2];
	i2c_master_transfer_t * i2c_xfer_ptr;

	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, SETHOUR_SEQ_ENABLE,
				pdTRUE,pdTRUE, portMAX_DELAY );

		for ( msg_cnt = 0; sethour_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = sethour_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE, portMAX_DELAY );

		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
		uart_pkg->uart_to_comm = cfg_struct->uart_calling;
		xQueueSend( cfg_struct->rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_DONE, pdTRUE,
				pdTRUE,	portMAX_DELAY );
		msg_size = uxQueueMessagesWaiting ( cfg_struct->rx_queue );

		uint8_t hour_real [ msg_size ];

		for ( msg_cnt = 0; msg_cnt < 8; msg_cnt++ )
		{
			xQueueReceive( cfg_struct->rx_queue, &uart_pkg, portMAX_DELAY );
			hour_real [ count_msg ] = uart_pkg->data;
			count_msg++;
			if(uart_pkg->data == ':')
			{
				count_msg--;
			}
			vPortFree ( uart_pkg );
		}

		for ( msg_cnt = 0; msg_cnt < 6; msg_cnt++ )
		{
			if(msg_cnt < 2)
			{
				hora[ msg_cnt ] = hour_real[ msg_cnt ]+0x00;
			}
			else if(msg_cnt >= 2 && msg_cnt < 4)
			{
				minutos[ msg_cnt-2 ] = hour_real[ msg_cnt ];
			}
			else if(msg_cnt >= 4)
			{
				segundos[ msg_cnt-4 ] = hour_real[ msg_cnt ];
			}
		}

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Write;
		i2c_xfer_ptr->subaddress = 0x02;
		i2c_xfer_ptr->subaddressSize = 2;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &segundos[ 0 ];
		i2c_xfer_ptr->dataSize = 1;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Write;
		i2c_xfer_ptr->subaddress = 0x03;
		i2c_xfer_ptr->subaddressSize = 2;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &minutos[ 0 ];
		i2c_xfer_ptr->dataSize = 1;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );
		vPortFree ( i2c_xfer_ptr );

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Write;
		i2c_xfer_ptr->subaddress = 0x04;
		i2c_xfer_ptr->subaddressSize = 1;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &hora[ 0 ];
		i2c_xfer_ptr->dataSize = 1;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );
		vPortFree ( i2c_xfer_ptr );

		for ( msg_cnt = 0; finish_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = finish_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE, portMAX_DELAY );

		xEventGroupSetBits ( cfg_struct->menu_event_handle, SETHOUR_SEQ_DONE );

	}
}

void setdate_sequence_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	i2c_master_transfer_t * i2c_xfer_ptr;
	uint8_t msg_cnt;
	uint8_t count_msg = 0;
	uint8_t setdate_msg [ ] = "\n\rEscribir fecha en formato dd/mm/aaaa\n\r\0";
	uint8_t finish_msg [ ] = "\n\rLa fecha cambio exitosamente...\n\r\0";

	uart_pkg_struct_t * uart_pkg;
	uint8_t msg_size;
	uint8_t dia[2];
	uint8_t mes[2];
	uint8_t anio[4];


	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, SETDATE_SEQ_ENABLE,
				pdTRUE,pdTRUE, portMAX_DELAY );

		for ( msg_cnt = 0; setdate_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = setdate_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE, portMAX_DELAY );

		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
		uart_pkg->uart_to_comm = cfg_struct->uart_calling;
		xQueueSend( cfg_struct->rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_DONE, pdTRUE,
				pdTRUE,	portMAX_DELAY );
		msg_size = uxQueueMessagesWaiting ( cfg_struct->rx_queue );

		uint8_t date_real [ msg_size ];

		for ( msg_cnt = 0; msg_cnt < 10; msg_cnt++ )
		{
			xQueueReceive( cfg_struct->rx_queue, &uart_pkg, portMAX_DELAY );
			date_real [ count_msg ] = uart_pkg->data;
			count_msg++;
			if(uart_pkg->data == '/')
			{
				count_msg--;
			}
			vPortFree ( uart_pkg );
		}

		for ( msg_cnt = 0; msg_cnt < 8; msg_cnt++ )
		{
			if(msg_cnt < 2)
			{
				dia[ msg_cnt ] = date_real[ msg_cnt ];
			}
			else if(msg_cnt >= 2 && msg_cnt < 4)
			{
				mes[ msg_cnt-2 ] = date_real[ msg_cnt ];
			}
			else if(msg_cnt >= 4)
			{
				anio[ msg_cnt-4 ] = date_real[ msg_cnt ];
			}
		}

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Write;
		i2c_xfer_ptr->subaddress = 0x05;
		i2c_xfer_ptr->subaddressSize = 1;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &dia[ 0 ];
		i2c_xfer_ptr->dataSize = 1;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Write;
		i2c_xfer_ptr->subaddress = 0x06;
		i2c_xfer_ptr->subaddressSize = 1;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &mes[ 0 ];
		i2c_xfer_ptr->dataSize = 1;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );

		for ( msg_cnt = 0; finish_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = finish_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE, portMAX_DELAY );

		xEventGroupSetBits ( cfg_struct->menu_event_handle, SETDATE_SEQ_DONE );
	}
}
void format_sequence_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	i2c_master_transfer_t * i2c_xfer_ptr;
	uint8_t msg_cnt;
	uint8_t msg_size;
	uint8_t hour12_msg [ ] = "\n\rEl formato acutal es 12H\n\r\0";
	uint8_t hour24_msg [ ] = "\n\rEl formato acutal es 24H\n\r\0";
	uint8_t cuestion_msg [ ] = "\n\rDesea cambiar el formato de hora? 1.si\n\r2.no\n\r\0";
	uint8_t without_msg [ ] = "\n\rNo se realizo ningun cambio...\n\r\0";
	uint8_t finish_msg [ ] = "\n\rEl formato ha sido cambiado...\n\r\0";
	uart_pkg_struct_t * uart_pkg;

	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, FORMAT_SEQ_ENABLE,
				pdTRUE,pdTRUE, portMAX_DELAY );

		if(0 == Flag12_24hour)
		{
			for ( msg_cnt = 0; hour12_msg [ msg_cnt ] != '\0'; msg_cnt++ )
			{
				uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
				uart_pkg->data = hour12_msg [ msg_cnt ];
				uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
				uart_pkg->uart_to_comm = cfg_struct->uart_calling;
				xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
			}
			xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
			xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
					pdTRUE, portMAX_DELAY );

			for ( msg_cnt = 0; cuestion_msg [ msg_cnt ] != '\0'; msg_cnt++ )
			{
				uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
				uart_pkg->data = cuestion_msg [ msg_cnt ];
				uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
				uart_pkg->uart_to_comm = cfg_struct->uart_calling;
				xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
			}
			xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
			xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
					pdTRUE, portMAX_DELAY );
			////////////////////////////////////
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = 0;
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->rx_cfg_queue, &uart_pkg, portMAX_DELAY );
			xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_ENABLE );
			xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_DONE, pdTRUE,
					pdTRUE,	portMAX_DELAY );
			msg_size = uxQueueMessagesWaiting ( cfg_struct->rx_queue );

			uint8_t answer [ msg_size ];

			for ( msg_cnt = 0; msg_cnt < msg_size; msg_cnt++ )
			{
				xQueueReceive( cfg_struct->rx_queue, &uart_pkg, portMAX_DELAY );
				answer [ msg_cnt ] = uart_pkg->data;
				vPortFree ( uart_pkg );
			}

			if ( '1' == answer[0] )
			{
				//cambiar formato
				for ( msg_cnt = 0; finish_msg [ msg_cnt ] != '\0'; msg_cnt++ )
				{
					uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
					uart_pkg->data = finish_msg [ msg_cnt ];
					uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
					uart_pkg->uart_to_comm = cfg_struct->uart_calling;
					xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
				}
				xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
				xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
						pdTRUE, portMAX_DELAY );
				Flag12_24hour = 1;
				xEventGroupSetBits ( cfg_struct->menu_event_handle, FORMAT_SEQ_DONE );

			}
			else
			{
				for ( msg_cnt = 0; without_msg [ msg_cnt ] != '\0'; msg_cnt++ )
				{
					uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
					uart_pkg->data = without_msg [ msg_cnt ];
					uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
					uart_pkg->uart_to_comm = cfg_struct->uart_calling;
					xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
				}
				xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
				xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
						pdTRUE, portMAX_DELAY );
				xEventGroupSetBits ( cfg_struct->menu_event_handle, FORMAT_SEQ_DONE );
			}
		}
		else if ( 1 == Flag12_24hour )
		{
			for ( msg_cnt = 0; hour24_msg [ msg_cnt ] != '\0'; msg_cnt++ )
			{
				uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
				uart_pkg->data = hour24_msg [ msg_cnt ];
				uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
				uart_pkg->uart_to_comm = cfg_struct->uart_calling;
				xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
			}
			xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
			xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
					pdTRUE, portMAX_DELAY );

			for ( msg_cnt = 0; cuestion_msg [ msg_cnt ] != '\0'; msg_cnt++ )
			{
				uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
				uart_pkg->data = cuestion_msg [ msg_cnt ];
				uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
				uart_pkg->uart_to_comm = cfg_struct->uart_calling;
				xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
			}
			xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
			xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
					pdTRUE, portMAX_DELAY );
			////////////////////////////////////
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = 0;
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->rx_cfg_queue, &uart_pkg, portMAX_DELAY );
			xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_ENABLE );
			xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_DONE, pdTRUE,
					pdTRUE,	portMAX_DELAY );
			msg_size = uxQueueMessagesWaiting ( cfg_struct->rx_queue );

			uint8_t answer [ msg_size ];

			for ( msg_cnt = 0; msg_cnt < msg_size; msg_cnt++ )
			{
				xQueueReceive( cfg_struct->rx_queue, &uart_pkg, portMAX_DELAY );
				answer [ msg_cnt ] = uart_pkg->data;
				vPortFree ( uart_pkg );
			}

			if ( '1' == answer[0] )
			{
				//cambiar formato
				for ( msg_cnt = 0; finish_msg [ msg_cnt ] != '\0'; msg_cnt++ )
				{
					uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
					uart_pkg->data = finish_msg [ msg_cnt ];
					uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
					uart_pkg->uart_to_comm = cfg_struct->uart_calling;
					xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
				}
				xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
				xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
						pdTRUE, portMAX_DELAY );
				Flag12_24hour = 0;
				xEventGroupSetBits ( cfg_struct->menu_event_handle, FORMAT_SEQ_DONE );

			}
			else
			{
				for ( msg_cnt = 0; without_msg [ msg_cnt ] != '\0'; msg_cnt++ )
				{
					uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
					uart_pkg->data = without_msg [ msg_cnt ];
					uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
					uart_pkg->uart_to_comm = cfg_struct->uart_calling;
					xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
				}
				xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
				xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
						pdTRUE, portMAX_DELAY );
				xEventGroupSetBits ( cfg_struct->menu_event_handle, FORMAT_SEQ_DONE );
			}
		}
	}
	xEventGroupSetBits ( cfg_struct->menu_event_handle, FORMAT_SEQ_DONE );
}

void readhour_sequence_task ( void * arg )
{
	i2c_master_transfer_t * i2c_xfer_ptr;
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uart_pkg_struct_t * uart_pkg;
	uint8_t read_hour;
	uint8_t read_min;
	uint8_t read_seg;
	uint8_t msg_cnt;
	uint8_t Read_hour [ ] = "\n\rLa hora exacta es:\n\r\0";

	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, READHOUR_SEQ_ENABLE,
				pdTRUE,pdTRUE, portMAX_DELAY );
		for ( msg_cnt = 0; Read_hour [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = Read_hour [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE, portMAX_DELAY );

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Read;
		i2c_xfer_ptr->subaddress = 0x04;
		i2c_xfer_ptr->subaddressSize = 1;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &read_hour;
		i2c_xfer_ptr->dataSize = 1;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Read;
		i2c_xfer_ptr->subaddress = 0x03;
		i2c_xfer_ptr->subaddressSize = 1;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &read_min;
		i2c_xfer_ptr->dataSize = 1;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Read;
		i2c_xfer_ptr->subaddress = 0x02;
		i2c_xfer_ptr->subaddressSize = 1;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &read_seg;
		i2c_xfer_ptr->dataSize = 1;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );


		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = read_hour;
		uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
		uart_pkg->uart_to_comm = cfg_struct->uart_calling;
		xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );

		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE, portMAX_DELAY );

		xEventGroupSetBits ( cfg_struct->menu_event_handle, READHOUR_SEQ_DONE );

	}
}
void readdate_sequence_task ( void * arg )
{
	i2c_master_transfer_t * i2c_xfer_ptr;
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uart_pkg_struct_t * uart_pkg;
	uint8_t read_day;
	uint8_t read_mo;
	uint8_t msg_cnt;
	uint8_t Read_date [ ] = "\n\rLa fecha exacta es:\n\r\0";
	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, READDATE_SEQ_ENABLE,
				pdTRUE,pdTRUE, portMAX_DELAY );

		for ( msg_cnt = 0; Read_date [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = Read_date [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = cfg_struct->uart_calling;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
				pdTRUE, portMAX_DELAY );

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Read;
		i2c_xfer_ptr->subaddress = 0x05;
		i2c_xfer_ptr->subaddressSize = 1;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &read_day;
		i2c_xfer_ptr->dataSize = 1;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );

		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Read;
		i2c_xfer_ptr->subaddress = 0x06;
		i2c_xfer_ptr->subaddressSize = 1;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &read_mo;
		i2c_xfer_ptr->dataSize = 1;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->menu_event_handle, READDATE_SEQ_DONE );
	}
}
void eco_sequence_task ( void * arg )
{
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, ECO_SEQ_ENABLE,
				pdTRUE,pdTRUE, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->menu_event_handle, ECO_SEQ_DONE );
	}
}
void GetTime_default_t(void * arg)
{
	i2c_master_transfer_t * i2c_xfer_ptr;
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
	uint8_t Counter = 0;
	uint8_t SubAddress = 0x02;
	uint8_t data_buffer = SET_SECONDS_DEFAULT;
	uint8_t buffer_0h = 0x08;

	i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
	i2c_xfer_ptr->slaveAddress = 0x50;
	i2c_xfer_ptr->direction = kI2C_Write;
	i2c_xfer_ptr->subaddress = 0x00;
	i2c_xfer_ptr->subaddressSize = 1;
	i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
	i2c_xfer_ptr->data = &buffer_0h;
	i2c_xfer_ptr->dataSize = 1;

	xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
	xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
	xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
			pdTRUE,portMAX_DELAY );

	for(Counter = 0; Counter < 5; Counter++)
	{
		i2c_xfer_ptr = pvPortMalloc ( sizeof(i2c_master_transfer_t) );
		i2c_xfer_ptr->slaveAddress = 0x50;
		i2c_xfer_ptr->direction = kI2C_Write;
		i2c_xfer_ptr->subaddress = SubAddress;
		i2c_xfer_ptr->subaddressSize = 1;
		i2c_xfer_ptr->flags = kI2C_TransferDefaultFlag;
		i2c_xfer_ptr->data = &data_buffer;
		i2c_xfer_ptr->dataSize = 1;

		xQueueSend( cfg_struct->i2c_queue, &i2c_xfer_ptr, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->i2c_event_handle, I2C_ENABLE );
		xEventGroupWaitBits ( cfg_struct->i2c_event_handle, I2C_DONE, pdTRUE,
				pdTRUE,portMAX_DELAY );

		SubAddress += 0x01;
		if(SubAddress == 0x03 )
		{
			data_buffer = SET_MINUTES_DEFAULT;
		}
		else if (SubAddress == 0x04)
		{
			data_buffer = SET_HOURS_DEFAULT;
		}
		else if (SubAddress == 0x05)    //year/date
		{
			data_buffer = 0x31;
		}
		else if (SubAddress == 0x06)       //mes
		{
			data_buffer = 0x12;
		}
	}
}
