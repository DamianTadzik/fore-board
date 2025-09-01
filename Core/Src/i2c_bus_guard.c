/*
 * i2c_bus_guard.c
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */

#include "i2c_bus_guard.h"

static osMutexId_t s_i2c1_mx = NULL;
static osMutexId_t s_i2c2_mx = NULL;

void i2c_mutexes_init(void)
{
    if (!s_i2c1_mx) s_i2c1_mx = osMutexNew(NULL);
    if (!s_i2c2_mx) s_i2c2_mx = osMutexNew(NULL);
}

osMutexId_t i2c1_mutex_get(void) { return s_i2c1_mx; }
osMutexId_t i2c2_mutex_get(void) { return s_i2c2_mx; }
