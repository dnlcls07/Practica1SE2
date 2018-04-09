/*
 * i2c_tasks.h
 *
 *  Created on: Mar 26, 2018
 *      Author: dceli
 */

#ifndef I2C_TASKS_H_
#define I2C_TASKS_H_

#include "definitions.h"
#include "FreeRTOS.h"

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#define I2C_ENABLE (1<<0)
#define I2C_DONE (1<<1)

i2c_master_handle_t * i2c_get_master_handle ( void );
i2c_master_config_t * i2c_get_i2c_master_config ( void );
SemaphoreHandle_t * i2c_get_i2c_semaphore ( void );

/********************************************************************************************/
/*!
 \brief
 I2C driver startup.
 \param[in] void
 \return void
 */
void i2c_init_task ( void * arg );
/********************************************************************************************/
/*!
 \brief
 I2C interrupt callback. Frees the semaphore.
 \param[in] void
 \return void
 */
void i2c_master_callback ( I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void *userData );
/********************************************************************************************/
/*!
 \brief
 I2C task. Retreives a transfer handle from its queue, where it is specified if it
 will read or write, the address and subaddress, and the data or place to receive.
 \param[in] void
 \return void
 */
void i2c_task ( void * arg );

#endif /* I2C_TASKS_H_ */
