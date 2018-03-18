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
#include "semphr.h"
#include "task.h"
#include "event_groups.h"

/******************************** ***********************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_DSPI_MASTER_BASEADDR SPI0
#define DSPI_MASTER_CLK_SRC DSPI0_CLK_SRC
#define DSPI_MASTER_CLK_FREQ CLOCK_GetFreq(DSPI0_CLK_SRC)
#define EXAMPLE_DSPI_MASTER_PCS_FOR_INIT kDSPI_Pcs0
#define EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0

#define EXAMPLE_DSPI_DEALY_COUNT 0XFFFFFU
#define TRANSFER_SIZE 1U         /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 500000U /*! Transfer baudrate - 500k */
#define PTD1_SCK 1
#define PTD2_SOUT 2
#define Tranfer_progress 1

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* DSPI user callback */
void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);
void DSPI_SendOneByte(uint8_t data);
/*******************************************************************************
 * Variables
 ******************************************************************************/
//uint8_t masterRxData[TRANSFER_SIZE] = {0U};
//uint8_t masterTxData[TRANSFER_SIZE] = {0U};

dspi_master_handle_t g_m_handle;
dspi_transfer_t masterXfer;
EventGroupHandle_t spi_semaphore;
EventGroupHandle_t spi_mutex;

volatile bool isTransferCompleted = false;

void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{
	if (status == kStatus_Success)
	{
		__NOP();
	}

	//isTransferCompleted = true;
	BaseType_t xHigherPriorityTaskWoken;
	xEventGroupSetBitsFromISR(spi_semaphore, Tranfer_progress, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void DSPI_SendOneByte(uint8_t data)
{

	masterXfer.txData = &data;
	masterXfer.rxData = NULL;
	masterXfer.dataSize = TRANSFER_SIZE;
	masterXfer.configFlags = kDSPI_MasterCtar0 | EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

	/* Wait transfer complete */
	xSemaphoreTake(spi_mutex,portMAX_DELAY);
	DSPI_MasterTransferNonBlocking(SPI0, &g_m_handle, &masterXfer);
	xSemaphoreGive(spi_mutex);

	xEventGroupWaitBits(spi_semaphore, Tranfer_progress, pdTRUE, pdTRUE, portMAX_DELAY);
	DSPI_StopTransfer(SPI0);
}

int main(void)
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
	DSPI_MasterTransferCreateHandle(SPI0, &g_m_handle, DSPI_MasterUserCallback, NULL);
	LCDNokia_init(); /*! Configuration function for the LCD */

	spi_semaphore = xEventGroupCreate();
//	xTaskCreate(spi_espera, "SPI task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-3, NULL);
	vTaskStartScheduler();

	for(;;) {
		LCDNokia_clear();/*! It clears the information printed in the LCD*/
		LCDNokia_gotoXY(25,2);
		LCDNokia_sendChar('2'); /*! It prints a character*/
		LCDNokia_sendChar('0'); /*! It prints a character*/
		LCDNokia_sendChar('1'); /*! It prints a character*/
		LCDNokia_sendChar('7'); /*! It prints a character*/

	}

	return 0;
}

