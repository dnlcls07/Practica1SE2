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
#include "definitions.h"
#include "menu_tasks.h"
#include "i2c_tasks.h"
#include "uart_tasks.h"

int main ( void )
{

	BOARD_InitBootPins ();
	BOARD_InitBootClocks ();
	BOARD_InitBootPeripherals ();
	BOARD_InitDebugConsole ();

//Task startup

	xTaskCreate ( i2c_init_task, "I2C_init", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES, NULL );
	xTaskCreate ( UART0_menu_init_task, "UART0_menu_init", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 1, NULL );
	xTaskCreate ( UART1_menu_init_task, "UART1_menu_init", configMINIMAL_STACK_SIZE, NULL,
	configMAX_PRIORITIES - 2, NULL );
	vTaskStartScheduler ();

	while ( 1 )
	{

	}
	return 0;
}
