/*
 * task_can_rx.c
 *
 *  Created on: Aug 31, 2025
 *      Author: brzan
 */
#include "task_can_rx.h"
#include "main.h"
#include "cmsis_os2.h"

#include "can-not/can_not.h"

void task_can_rx(void *argument)
{
	cant_generic_struct_t tmp = { 0 };
	while (1)
	{
		if (MESSAGE_RECEIVED == cant_receive(&tmp))
		{
			//Process your received tmp message here...
			int x = 0;
			UNUSED(x);
		}
		osThreadYield();	//<- preffered no delay inside task loop in freeRTOS
	}
}


