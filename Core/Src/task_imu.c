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

//#include "can-not/can_not.h"
//#include "can-messages-mini-celka/src/cmmc.h"


extern volatile uint32_t task_imu_alive;
void task_imu(void *argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();
	(void)fb_ptr;

	while (1)
	{
		osDelay(10);
		task_imu_alive++;
	}
}
