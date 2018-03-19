/*
 * SPI.LCD.h
 *
 *  Created on: Mar 18, 2018
 *      Author: mango
 */

#ifndef SPI_LCD_H_
#define SPI_LCD_H_

#include "fsl_dspi.h"

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
void SPI_Init_t();


#endif /* SPI_LCD_H_ */
