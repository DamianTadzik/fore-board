/*
 * driver_ina226_interface.c
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */


#include "driver_ina226_interface.h"
#include <stdarg.h>
#include <stdio.h>

/*
 * Ten interfejs w libce ma funkcje globalne (bez "handle" w sygnaturach),
 * więc robimy prosty globalny kontekst "aktywny I2C". Przed KAŻDYM wywołaniem
 * funkcji drivera dla danego sensora ustawiamy ten kontekst.
 * UŻYWAJ: ina226_interface_set_hal(&hi2cX, i2cX_mutex);
 */

typedef struct {
    I2C_HandleTypeDef *hi2c;
    osMutexId_t        i2c_mutex;
} _ina226_hal_ctx_t;

static _ina226_hal_ctx_t _ctx = {0};

void ina226_interface_set_hal(I2C_HandleTypeDef *hi2c, osMutexId_t i2c_mutex)
{
    _ctx.hi2c = hi2c;
    _ctx.i2c_mutex = i2c_mutex;
}

/* Implementacje wymagane przez driver (podpisy z driver_ina226_interface.h) */

uint8_t ina226_interface_iic_init(void)
{
    /* CubeMX robi init HALa; tu nic nie rób. */
    return 0;
}

uint8_t ina226_interface_iic_deinit(void)
{
    return 0;
}

uint8_t ina226_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (_ctx.i2c_mutex) osMutexAcquire(_ctx.i2c_mutex, osWaitForever);
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(_ctx.hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
    if (_ctx.i2c_mutex) osMutexRelease(_ctx.i2c_mutex);
    return (st == HAL_OK) ? 0 : 1;
}

uint8_t ina226_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (_ctx.i2c_mutex) osMutexAcquire(_ctx.i2c_mutex, osWaitForever);
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(_ctx.hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
    if (_ctx.i2c_mutex) osMutexRelease(_ctx.i2c_mutex);
    return (st == HAL_OK) ? 0 : 1;
}

void ina226_interface_delay_ms(uint32_t ms)
{
    osDelay(ms);
}

void ina226_interface_debug_print(const char *const fmt, ...)
{
    va_list args; va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

/* Nie używamy receive_callback do CNVR, bo flagę i tak łapiemy EXTI+task.
   Zostawiamy pustą (albo można tu podłączyć własny mechanizm, jeśli kiedyś
   wywołasz ina226_irq_handler()) */
void ina226_interface_receive_callback(uint8_t type)
{
    (void)type;
}
