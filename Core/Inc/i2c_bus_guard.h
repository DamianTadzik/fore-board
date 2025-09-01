/*
 * i2c_bus_guard.h
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */

#ifndef INC_I2C_BUS_GUARD_H_
#define INC_I2C_BUS_GUARD_H_

#include "cmsis_os2.h"

void        i2c_mutexes_init(void);
osMutexId_t i2c1_mutex_get(void);
osMutexId_t i2c2_mutex_get(void);

#endif /* INC_I2C_BUS_GUARD_H_ */
