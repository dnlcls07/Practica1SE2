/*
 * menu_tasks.c
 *
 *  Created on: Mar 26, 2018
 *      Author: dceli
 */

#include "menu_tasks.h"

void Menu_init_task ( void * arg )
{
	menu_cfg_struct_t UART0_menu;
	menu_cfg_struct_t * UART0_cfg_menu_ptr = &UART0_menu;

	UART0_menu.addr_queue = xQueueCreate( 1, sizeof(uint16_t) );
	UART0_menu.bcd_queue = xQueueCreate( 1, sizeof(uint16_t) );
	UART0_menu.i2c_event_handle = xEventGroupCreate ();
	UART0_menu.menu_event_handle = xEventGroupCreate ();
	UART0_menu.uart_event_handle = xEventGroupCreate ();
	UART0_menu.uart_handle = &uart_pc_handle;
	UART0_menu.uart_calling = UART0;
	UART0_menu.rx_queue = xQueueCreate( UART_BUFFER_SIZE, sizeof(void*) );
	UART0_menu.tx_queue = xQueueCreate( UART_BUFFER_SIZE, sizeof(void*) );
	UART0_menu.i2c_queue = xQueueCreate( MAX_I2C_PAGE_SIZE, sizeof(void*) );
	UART0_menu.rx_cfg_queue = xQueueCreate( 1, sizeof(void*) );

	xTaskCreate ( esc_sequence_task, "ESCTask", configMINIMAL_STACK_SIZE,
			( void * ) UART0_cfg_menu_ptr, configMAX_PRIORITIES, NULL );
	xTaskCreate ( menu_sequence_task, "MenuTask", configMINIMAL_STACK_SIZE,
			( void * ) UART0_cfg_menu_ptr, configMAX_PRIORITIES - 1, NULL );

	xTaskCreate ( write_sequence_task, "WRtask", configMINIMAL_STACK_SIZE,
			( void * ) UART0_cfg_menu_ptr, configMAX_PRIORITIES - 2, NULL );
	xTaskCreate ( read_sequence_task, "RDtask", configMINIMAL_STACK_SIZE,
			( void * ) UART0_cfg_menu_ptr, configMAX_PRIORITIES - 2, NULL );

	xTaskCreate ( UART0_init_task, "UART_init", configMINIMAL_STACK_SIZE,
			( void * ) UART0_cfg_menu_ptr, configMAX_PRIORITIES, NULL );
	xTaskCreate ( i2c_init_task, "I2C_Init", configMINIMAL_STACK_SIZE,
			( void * ) UART0_cfg_menu_ptr, configMAX_PRIORITIES, NULL );

	xTaskCreate ( addr_parser_task, "ADDRtask", configMINIMAL_STACK_SIZE,
			( void * ) UART0_cfg_menu_ptr, configMAX_PRIORITIES - 4, NULL );
	xTaskCreate ( bcd_parser_task, "BCDTask", configMINIMAL_STACK_SIZE,
			( void * ) UART0_cfg_menu_ptr, configMAX_PRIORITIES - 4, NULL );

	vTaskDelay ( portMAX_DELAY );

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
		pdTRUE,
		pdTRUE,
		portMAX_DELAY );
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
		pdTRUE, pdTRUE,
		portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->menu_event_handle, ESC_B2MENU );
	}
}

void menu_sequence_task ( void * arg )
{
	vTaskDelete ( Menu_init_task );
	vTaskDelete ( i2c_init_task );
	vTaskDelete ( UART0_init_task );
	menu_cfg_struct_t * cfg_struct = ( menu_cfg_struct_t * ) arg;
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
		uart_pkg->uart_to_comm = UART0;
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
			default :
				break;
		}
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
		pdTRUE,
		pdTRUE, portMAX_DELAY );

		for ( msg_cnt = 0; read_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = read_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = UART0;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
		pdTRUE,
		portMAX_DELAY );
		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
		uart_pkg->uart_to_comm = UART0;
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
			i2c_pkg->uart_handle_to_comm = &uart_pc_handle;
			i2c_pkg->uart_to_comm = UART0;
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

	uint8_t msg_size;
	uint16_t addr;
	i2c_master_transfer_t * i2c_xfer_ptr;

	for ( ;; )
	{
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, WRITE_SEQ_ENABLE,
		pdTRUE,
		pdTRUE, portMAX_DELAY );
		for ( msg_cnt = 0; read_msg [ msg_cnt ] != '\0'; msg_cnt++ )
		{
			uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
			uart_pkg->data = read_msg [ msg_cnt ];
			uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
			uart_pkg->uart_to_comm = UART0;
			xQueueSend( cfg_struct->tx_queue, &uart_pkg, portMAX_DELAY );
		}
		xEventGroupSetBits ( cfg_struct->uart_event_handle, TX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, TX_DONE, pdTRUE,
		pdTRUE,
		portMAX_DELAY );
		uart_pkg = pvPortMalloc ( sizeof(uart_pkg_struct_t) );
		uart_pkg->data = 0;
		uart_pkg->uart_handle_to_comm = cfg_struct->uart_handle;
		uart_pkg->uart_to_comm = UART0;
		xQueueSend( cfg_struct->rx_cfg_queue, &uart_pkg, portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->uart_event_handle, RX_ENABLE );
		xEventGroupWaitBits ( cfg_struct->uart_event_handle, RX_DONE, pdTRUE,
		pdTRUE,
		portMAX_DELAY );
		xEventGroupSetBits ( cfg_struct->menu_event_handle, ADDR_ENABLE );
		xEventGroupWaitBits ( cfg_struct->menu_event_handle, ADDR_DONE, pdTRUE,
		pdTRUE,
		portMAX_DELAY );
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
		pdTRUE,
		portMAX_DELAY );
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
			xQueueReceive( cfg_struct->rx_queue, &msg_received [ msg_cnt ],
					portMAX_DELAY );
		}
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
		pdTRUE,
		portMAX_DELAY );

		xEventGroupSetBits ( cfg_struct->menu_event_handle, WRITE_SEQ_DONE );
	}
}

