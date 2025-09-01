/*
 * task_servo_power_monitor.c
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */

#include "task_servo_power_monitor.h"
#include "main.h"
#include "cmsis_os2.h"
#include "fore_board.h"

#include "i2c.h"
#include "i2c_bus_guard.h"
#include "INA226/driver_ina226.h"

/* --- Lokalne singletons dla taska --- */
static servo_ina_t  g_ina1, g_ina2;
static osThreadId_t g_powerTaskTid;

/*
 * Inicjalizacja jednego sensora:
 * - link funkcji interfejsu (DRIVER_INA226_LINK_*)
 * - ustawienie adresu, czasów, averagingu, trybu
 * - włączenie alertu Conversion Ready (CNVR)
 * - kalibracja (TU ustaw prawdziwe wartości!)
 */
static int ina_setup_one(servo_ina_t *n,
                         I2C_HandleTypeDef *hi2c, osMutexId_t bus_mx,
                         ina226_address_t addr,
                         GPIO_TypeDef *alert_port, uint16_t alert_pin, uint32_t flag_bit)
{
    n->hi2c      = hi2c;
    n->i2c_mutex = bus_mx;
    n->i2c_addr  = addr;
    n->alert_port= alert_port;
    n->alert_pin = alert_pin;
    n->flag_bit  = flag_bit;

    /* Powiązanie funkcji interfejsu (podpisy i makra – wg driver_ina226.h) */
    DRIVER_INA226_LINK_INIT(&n->drv, ina226_handle_t);
    DRIVER_INA226_LINK_IIC_INIT(&n->drv,    ina226_interface_iic_init);
    DRIVER_INA226_LINK_IIC_DEINIT(&n->drv,  ina226_interface_iic_deinit);
    DRIVER_INA226_LINK_IIC_READ(&n->drv,    ina226_interface_iic_read);
    DRIVER_INA226_LINK_IIC_WRITE(&n->drv,   ina226_interface_iic_write);
    DRIVER_INA226_LINK_DELAY_MS(&n->drv,    ina226_interface_delay_ms);
    DRIVER_INA226_LINK_DEBUG_PRINT(&n->drv, ina226_interface_debug_print);
    DRIVER_INA226_LINK_RECEIVE_CALLBACK(&n->drv, ina226_interface_receive_callback);

    /* Ustaw aktywną magistralę I2C dla operacji na TYM sensorze */
    ina226_interface_set_hal(n->hi2c, n->i2c_mutex);

    /* Info (opcjonalnie) i init */
    if (ina226_info(&n->info) != 0) return -1;
    ina226_set_addr_pin(&n->drv, n->i2c_addr);
    if (ina226_init(&n->drv) != 0)  return -2;

    /* Konfiguracja pomiarowa */
    ina226_set_average_mode(&n->drv, INA226_AVG_16);
    ina226_set_bus_voltage_conversion_time(&n->drv,   INA226_CONVERSION_TIME_1P1_MS);
    ina226_set_shunt_voltage_conversion_time(&n->drv, INA226_CONVERSION_TIME_1P1_MS);
    ina226_set_mode(&n->drv, INA226_MODE_SHUNT_BUS_VOLTAGE_CONTINUOUS);

    /* KALIBRACJA – PODAJ SWOJE Rshunt i policz/cal wpisz.
       Prosty wariant z gotowcem (jeśli używasz API "calculate"): */
     ina226_set_resistance(&n->drv, 0.082);            // <-- Rshunt [ohm]
     uint16_t cal;
     ina226_calculate_calibration(&n->drv, &cal);
     ina226_set_calibration(&n->drv, cal);

    /* albo jeśli wolisz ręczny dobór "cal": */
    // ina226_set_resistance(&n->drv, 0.05);
    // ina226_set_calibration(&n->drv, 1024);           // PRZYKŁAD – zmień na swoje!

    /* ALERT = Conversion Ready (CNVR). Dla CNVR LIMIT nie jest wymagany. */
    ina226_set_conversion_ready_alert_pin(&n->drv, INA226_BOOL_TRUE);

    return 0;
}

void exti_task_servo_power_monitor_callback(uint16_t GPIO_Pin)
{
    uint32_t flags = 0;
    if (GPIO_Pin == g_ina1.alert_pin) flags |= g_ina1.flag_bit;
    if (GPIO_Pin == g_ina2.alert_pin) flags |= g_ina2.flag_bit;

    if (flags && g_powerTaskTid) osThreadFlagsSet(g_powerTaskTid, flags);
}

extern volatile uint32_t task_servo_power_monitor_alive;
void task_servo_power_monitor(void *argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();
	g_powerTaskTid = osThreadGetId();

    /* INIT dwóch czujników
       - podstaw Swoje piny EXTI oraz faktyczne adresy A0/A1:
         UWAGA: enumy adresów w driverze są JUŻ przesunięte <<1, zgodne z HALem! */
    int r1 = ina_setup_one(&g_ina1, &hi2c1, i2c1_mutex_get(), INA226_ADDRESS_0, EXTI_INA_1_GPIO_Port, EXTI_INA_1_Pin, INA1_FLAG);
    int r2 = ina_setup_one(&g_ina2, &hi2c2, i2c2_mutex_get(), INA226_ADDRESS_0, EXTI_INA_2_GPIO_Port, EXTI_INA_2_Pin, INA2_FLAG);
    UNUSED(r1);
    UNUSED(r2);

	while (1)
	{
        /* Czekaj na CNVR z któregoś sensora (albo timeout jako watchdog) */
        uint32_t flags = osThreadFlagsWait(INA1_FLAG | INA2_FLAG, osFlagsWaitAny, 250);

        /* Aktywuj kontekst I2C #1 i czytaj wyniki.
           Funkcje read_* z drivera i tak na początku czytają MASK,
           co kasuje CNVR i zwalnia pin ALERT. */

        if (flags & INA1_FLAG)
        {
            ina226_interface_set_hal(g_ina1.hi2c, g_ina1.i2c_mutex);

            uint16_t vbus_raw, pwr_raw;
            int16_t  ish_raw;
            float    vbus_mV, ish_mA, pwr_mW;

            (void)ina226_read_bus_voltage(&g_ina1.drv, &vbus_raw, &vbus_mV);
            (void)ina226_read_current(&g_ina1.drv, &ish_raw, &ish_mA);
            (void)ina226_read_power(&g_ina1.drv, &pwr_raw, &pwr_mW);


            fb_ptr->left_servo_power.current = ish_mA;
            fb_ptr->left_servo_power.voltage = vbus_mV;
        }

        if (flags & INA2_FLAG)
        {
            ina226_interface_set_hal(g_ina2.hi2c, g_ina2.i2c_mutex);

            uint16_t vbus_raw, pwr_raw;
            int16_t  ish_raw;
            float    vbus_mV, ish_mA, pwr_mW;

            (void)ina226_read_bus_voltage(&g_ina2.drv, &vbus_raw, &vbus_mV);
            (void)ina226_read_current(&g_ina2.drv, &ish_raw, &ish_mA);
            (void)ina226_read_power(&g_ina2.drv, &pwr_raw, &pwr_mW);

            fb_ptr->right_servo_power.current = ish_mA;
            fb_ptr->right_servo_power.voltage = vbus_mV;
        }

        /* opcjonalnie: gdy timeout – możesz zrobić sanity poll raz na jakiś czas */

        task_servo_power_monitor_alive++;
	}
}
