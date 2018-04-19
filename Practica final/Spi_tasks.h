/*
 * Spi_tasks.h
 *
 *  Created on: Apr 9, 2018
 *      Author: Gustavo
 */

#ifndef SPI_TASKS_H_
#define SPI_TASKS_H_

#include "definitions.h"
#include "FreeRTOS.h"

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_dspi.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"
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

#define SPI_ENABLE (1<<0)
#define SPI_DONE (1<<1)

dspi_master_handle_t * spi_get_master_handle ( void );
dspi_master_config_t * spi_get_dspi_master_config ( void );
SemaphoreHandle_t * spi_get_dspi_semaphore ( void );

/********************************************************************************************/
/*!
 \brief
 SPI driver startup.
 \param[in] void
 \return void
 */
void SPI_init_task ( void * arg );
/********************************************************************************************/
/*!
 \brief
 SPI interrupt callback. Frees the semaphore.
 \param[in] void
 \return void
 */
void DSPI_master_callback ( I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void *userData );
/********************************************************************************************/
/*!
 \brief
 SPI task. Retreives a transfer handle from its queue, where it is specified if it
 will read or write, the address and subaddress, and the data or place to receive.
 \param[in] void
 \return void
 */
void spi_task ( void * arg );

#endif /* SPI_TASKS_H_ */
