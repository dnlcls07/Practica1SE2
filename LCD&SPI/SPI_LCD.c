
/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    SPI_Freertos.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "FreeRTOS.h"
#include "LCDNokia5110.h"
#include "GlobalFunctions.h"
#include "SPI_LCD.h"


dspi_master_handle_t g_m_handle_SPI;
dspi_transfer_t masterXfer_SPI;

volatile bool isTransferCompleted = false;

void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{
	if (status == kStatus_Success)
	{
		__NOP();
	}

	isTransferCompleted = true;
}

void DSPI_SendOneByte(uint8_t data)
{
	isTransferCompleted = false;
	masterXfer_SPI.txData = &data;
	masterXfer_SPI.rxData = NULL;
	masterXfer_SPI.dataSize = TRANSFER_SIZE;
	masterXfer_SPI.configFlags = kDSPI_MasterCtar0 | EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;
	DSPI_MasterTransferNonBlocking(SPI0, &g_m_handle_SPI, &masterXfer_SPI);
	/* Wait transfer complete */
	while (!isTransferCompleted)
	{
	}

}

void SPI_Init_t()
{
	dspi_master_config_t masterConfig;
	uint32_t srcClock_Hz;

	CLOCK_EnableClock(kCLOCK_PortD);
	CLOCK_EnableClock(kCLOCK_Spi0);
	port_pin_config_t config_SPI =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt2,
			kPORT_UnlockRegister, };
	PORT_SetPinConfig(PORTD, PTD1_SCK, &config_SPI);
	PORT_SetPinConfig(PORTD, PTD2_SOUT, &config_SPI);

	DSPI_MasterGetDefaultConfig(&masterConfig);
	srcClock_Hz = DSPI_MASTER_CLK_FREQ;
	DSPI_MasterInit(SPI0,&masterConfig,srcClock_Hz);
	NVIC_EnableIRQ(SPI0_IRQn);
	NVIC_SetPriority(SPI0_IRQn,7);
	DSPI_MasterTransferCreateHandle(SPI0, &g_m_handle_SPI, DSPI_MasterUserCallback, NULL);
	LCDNokia_init(); /*! Configuration function for the LCD */
}
