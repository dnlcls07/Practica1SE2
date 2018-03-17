/*
 * GlobalFunctions.c
 *
 *  Created on: 16/08/2017
 *      Author: jlpe
 */

#include "GlobalFunctions.h"
#include "fsl_port.h"
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function is the delay
 */
void delay_t(uint16_t delayt)
{
	volatile int counter, counter2;

	for(counter2=16; counter2 > 0; counter2--)
	{
		for(counter=delayt; counter > 0; counter--);

	}
}
