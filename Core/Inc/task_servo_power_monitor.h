/*
 * task_servo_power_monitor.h
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */

#ifndef INC_TASK_SERVO_POWER_MONITOR_H_
#define INC_TASK_SERVO_POWER_MONITOR_H_

#include "INA226/driver_ina226.h"
#include "INA226/driver_ina226_interface.h"
#include "cmsis_os2.h"
#include "main.h"

/* Flagi do powiadamiania taska */
#define INA1_FLAG   (1u << 0)
#define INA2_FLAG   (1u << 1)

/* Prosty opis jednego egzemplarza czujnika */
typedef struct {
    ina226_handle_t   drv;
    ina226_info_t     info;
    /* pin ALERT (EXTI) do rozróżnienia w ISR */
    GPIO_TypeDef     *alert_port;
    uint16_t          alert_pin;
    uint32_t          flag_bit;
    /* adres I2C wg enumów z drivera (UWAGA: one są JUŻ przesunięte <<1) */
    ina226_address_t  i2c_addr;
    /* który HAL I2C i mutex ma być aktywowany przy operacjach */
    I2C_HandleTypeDef *hi2c;
    osMutexId_t        i2c_mutex;
} servo_ina_t;

void exti_task_servo_power_monitor_callback(uint16_t GPIO_Pin);

void task_servo_power_monitor(void *argument);

#endif /* INC_TASK_SERVO_POWER_MONITOR_H_ */
