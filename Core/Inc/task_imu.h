/*
 * task_imu.h
 *
 *  Created on: Nov 2, 2025
 *      Author: brzan
 */

#ifndef INC_TASK_IMU_H_
#define INC_TASK_IMU_H_

#include "main.h"

void exti_task_imu_callback(uint16_t GPIO_Pin);

void task_imu(void *argument);

#endif /* INC_TASK_IMU_H_ */
