/*
 * task_imu.c
 *
 *  Created on: Nov 2, 2025
 *      Author: brzan
 */

#include "task_imu.h"
#include "main.h"
#include "cmsis_os2.h"
#include "fore_board.h"

#include "spi.h"
#include "MPU9250/MPU9250.h"

MPU9250_t MPU9250;

extern volatile uint32_t task_imu_alive;
void task_imu(void *argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();
	(void)fb_ptr;

	MPU9250.settings.gFullScaleRange = GFSR_500DPS;
	MPU9250.settings.aFullScaleRange = AFSR_4G;
	MPU9250.settings.CS_PIN = SPI1_CS_IMU_Pin;
	MPU9250.settings.CS_PORT = SPI1_CS_IMU_GPIO_Port;
	MPU9250.attitude.tau = 0.98;
	MPU9250.attitude.dt = 0.004;

	osDelay(100);

    // Startup / reset the sensor
	uint8_t addr, val;
    addr = PWR_MGMT_1;
    val = 0x00;
    MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &val);

	// Check if IMU configured properly and block if it didn't
	if (MPU_begin(&hspi1, &MPU9250) != 1)
	{
//		sprintf((char *)serialBuf, "ERROR!\r\n");
//		HAL_UART_Transmit(&huart2, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
		while (1){ osDelay(1000); }
	}

	// Calibrate the IMU
	MPU_calibrateGyro(&hspi1, &MPU9250, 1500);

//	// Start timer and put processor into an efficient low power mode
//	HAL_TIM_Base_Start_IT(&htim11);
//	HAL_PWR_EnableSleepOnExit();
//	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

	while (1)
	{

		osDelay(1000);
		task_imu_alive++;
	}
}
