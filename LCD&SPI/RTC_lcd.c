#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
#include "GlobalFunctions.h"
#include "RTC_init.h"
#include "SPI_LCD.h"


int main(void)
{
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
	RTC_Init_t();
	GetTime_default_t();
	SPI_Init_t();
	while (1)
	{
		LCD_SetDate();
	}
	return 0;
}
