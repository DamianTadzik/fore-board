/*
 * task_range_meas.h
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */

#ifndef INC_TASK_RANGE_MEAS_H_
#define INC_TASK_RANGE_MEAS_H_

#include "main.h"

void exti_task_range_meas_callback(uint16_t GPIO_Pin);

void task_range_meas(void *argument);

#endif /* INC_TASK_RANGE_MEAS_H_ */
