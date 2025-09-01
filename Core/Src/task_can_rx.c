/*
 * task_can_rx.c
 *
 *  Created on: Aug 31, 2025
 *      Author: brzan
 */
#include "task_can_rx.h"
#include "main.h"
#include "cmsis_os2.h"
#include "fore_board.h"

#include "can-not/can_not.h"
//#include "canmsgs


extern volatile uint32_t task_can_rx_alive;
void task_can_rx(void *argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();

	cant_generic_struct_t tmp = { 0 };
	while (1)
	{
		if (MESSAGE_RECEIVED == cant_receive(&tmp))
		{
			//Process your received tmp message here...
			int x = 0;
			UNUSED(x);
		}
		task_can_rx_alive++;
		osThreadYield();	//<- preffered no delay inside this task loop in freeRTOS
	}
}


