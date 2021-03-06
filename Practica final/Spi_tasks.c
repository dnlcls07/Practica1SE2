/*
 * Spi_tasks.c
 *
 *  Created on: Apr 1, 2018
 *      Author: Gustavo Araiza
 */

#include "Spi_tasks.h"
#include "LCDNokia5110.h"

//variables
dspi_master_handle_t g_m_handle_SPI;
dspi_master_config_t masterConfig_SPI;
dspi_transfer_t masterXfer_SPI;
SemaphoreHandle_t SPI_semaphore;
SemaphoreHandle_t SPI_mutex;

volatile bool isTransferCompleted = false;

dspi_master_handle_t * spi_get_master_handle ( void )
{
	return &g_m_handle_SPI;
}

dspi_master_config_t * spi_get_dspi_master_config ( void )
{
	return &masterConfig_SPI;
}

SemaphoreHandle_t * spi_get_dspi_semaphore ( void )
{
	return &SPI_semaphore;
}

void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{

	//	BaseType_t xHigherPriorityTaskWoken;
	//	xHigherPriorityTaskWoken = pdFALSE;
	//	if (kStatus_Success == status)
	//	{
	//		xSemaphoreGiveFromISR( SPI_semaphore, &xHigherPriorityTaskWoken );
	//	}
	//	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	//	userData = userData;
	//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (status == kStatus_Success)
	{
		//xSemaphoreGiveFromISR(Transfer_Dspi_Semaphore, &xHigherPriorityTaskWoken);
		__NOP();
	}
	//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	isTransferCompleted = true;
}
void DSPI_SendOneByte(uint8_t data)
{
	isTransferCompleted = false;
	masterXfer_SPI.txData = &data;
	masterXfer_SPI.rxData = NULL;
	masterXfer_SPI.dataSize = TRANSFER_SIZE;
	masterXfer_SPI.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

	DSPI_MasterTransferNonBlocking(SPI0, &g_m_handle_SPI, &masterXfer_SPI);
	/* Wait transfer complete */
	//xSemaphoreTake(Transfer_Dspi_Semaphore, portMAX_DELAY);
	while (!isTransferCompleted)
	{
	}
}
void SPI_init_task ( void * arg )
{
	uint32_t srcClock_Hz;
	CLOCK_EnableClock(kCLOCK_PortD);
	CLOCK_EnableClock(kCLOCK_Spi0);

	port_pin_config_t config_SPI =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAlt2,
			kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTD, PTD1_SCK, &config_SPI);
	PORT_SetPinConfig(PORTD, PTD2_SOUT, &config_SPI);

	DSPI_MasterGetDefaultConfig(&masterConfig_SPI);
	srcClock_Hz = DSPI_MASTER_CLK_FREQ;
	DSPI_MasterInit(SPI0,&masterConfig_SPI,srcClock_Hz);
	DSPI_MasterTransferCreateHandle(SPI0, &g_m_handle_SPI,
			DSPI_MasterUserCallback, NULL);

	NVIC_EnableIRQ(SPI0_IRQn);
	NVIC_SetPriority(SPI0_IRQn,8);

	SPI_semaphore = xSemaphoreCreateBinary();
	SPI_mutex = xSemaphoreCreateMutex();
	LCDNokia_init();
	vTaskDelete ( NULL );
}
