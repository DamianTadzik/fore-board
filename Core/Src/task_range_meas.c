/*
 * task_range_meas.c
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */
#include "task_range_meas.h"
#include "main.h"
#include "cmsis_os2.h"
#include "fore_board.h"

#include "i2c.h"
#include "i2c_bus_guard.h"
#include "vl6180/vl6180_api.h"
#include "vl6180/vl6180_def.h"

#define TOF1_FLAG  (1u<<0)
#define TOF2_FLAG  (1u<<1)

static struct MyVL6180Dev_t tof1, tof2;
static osThreadId_t tof_thread;

void exti_task_range_meas_callback(uint16_t GPIO_Pin)
{
    if (!tof_thread) return;
    if (GPIO_Pin == EXTI_VL_1_Pin) osThreadFlagsSet(tof_thread, TOF1_FLAG);
    if (GPIO_Pin == EXTI_VL_2_Pin) osThreadFlagsSet(tof_thread, TOF2_FLAG);
}

extern volatile uint32_t task_range_meas_alive;
void task_range_meas(void *argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();
	tof_thread = osThreadGetId();

	HAL_GPIO_WritePin(GPIO_SHDN_VL_1_GPIO_Port, GPIO_SHDN_VL_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_SHDN_VL_2_GPIO_Port, GPIO_SHDN_VL_2_Pin, GPIO_PIN_RESET);
	int status = 0;

    // --- instancja #1 (I2C1) ---
    tof1.hi2c = &hi2c1;
    tof1.i2c_mx = i2c1_mutex_get();
    tof1.i2c_addr = 0x52;
    HAL_GPIO_WritePin(GPIO_SHDN_VL_1_GPIO_Port, GPIO_SHDN_VL_1_Pin, GPIO_PIN_SET);
    osDelay(20);
//    VL6180_WaitDeviceBooted(&tof1);
    status |= VL6180_InitData(&tof1);
    status |= VL6180_Prepare(&tof1);
    status |= VL6180_RangeSetInterMeasPeriod(&tof1, 100);
    status |= VL6180_FilterSetState(&tof1, 0);
    status |= VL6180_SetupGPIO1(&tof1, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_LOW);
    status |= VL6180_RangeConfigInterrupt(&tof1, CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);
    status |= VL6180_ClearAllInterrupt(&tof1);
    status |= VL6180_RangeStartContinuousMode(&tof1);
    status = 0;

    // --- instancja #2 (I2C2) ---
    tof2.hi2c = &hi2c2;
    tof2.i2c_mx = i2c2_mutex_get();
    tof2.i2c_addr = 0x52;
    HAL_GPIO_WritePin(GPIO_SHDN_VL_2_GPIO_Port, GPIO_SHDN_VL_2_Pin, GPIO_PIN_SET);
    osDelay(20);
//    VL6180_WaitDeviceBooted(&tof2);
    status |= VL6180_InitData(&tof2);
    status |= VL6180_Prepare(&tof2);
    status |= VL6180_RangeSetInterMeasPeriod(&tof2, 100);
    status |= VL6180_FilterSetState(&tof2, 0);
    status |= VL6180_SetupGPIO1(&tof2, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_LOW);
    status |= VL6180_RangeConfigInterrupt(&tof2, CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);
    status |= VL6180_ClearAllInterrupt(&tof2);
    status |= VL6180_RangeStartContinuousMode(&tof2);
    UNUSED(status);

    VL6180_RangeData_t d;

	while (1)
	{
        uint32_t f = osThreadFlagsWait(TOF1_FLAG|TOF2_FLAG, osFlagsWaitAny, 200);

        if (f & TOF1_FLAG) {
            VL6180_RangeGetMeasurement(&tof1, &d);
            VL6180_RangeClearInterrupt(&tof1);

            fb_ptr->left_tof.range_mm = d.range_mm;
        }

        if (f & TOF2_FLAG) {
            VL6180_RangeGetMeasurement(&tof2, &d);
            VL6180_RangeClearInterrupt(&tof2);

            fb_ptr->right_tof.range_mm = d.range_mm;
        }

        // można też w timeout zrobić sanity poll: VL6180_RangeGetMeasurementIfReady(...)

        task_range_meas_alive++;
	}
}
