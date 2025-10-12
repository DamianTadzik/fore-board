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
#include "can-messages-mini-celka/src/cmmc.h"

volatile uint32_t g_can_messages_arrived = 0;
volatile uint32_t g_can_messages_decoded = 0;


extern volatile uint32_t task_can_rx_alive;
void task_can_rx(void *argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();

	cant_generic_struct_t msg = { 0 };
	while (1)
	{
		if (MESSAGE_RECEIVED == cant_receive(&msg))
		{
			//Process your received tmp message here...
			switch (msg.msg_id)
			{
//			case 0x001:
//				// TODO FIXME decode this properly via dbc and add everything whats needed to ab structure xd
//				debug_g_tmp(&tmp);
//
//				ab_ptr->odesc.axis_current_state = l_tmp.msg_payload[4];
//
//				g_can_messages_decoded++;
//				break;

			case CMMC_RADIO_CONTROL_FRAME_ID:
				struct cmmc_radio_control_t tmp = { 0 };
				cmmc_radio_control_unpack(&tmp, msg.msg_payload, CMMC_RADIO_CONTROL_LENGTH);

				fb_ptr->from_radio.throttle = cmmc_radio_control_throttle_decode(tmp.throttle);
				fb_ptr->from_radio.steering = cmmc_radio_control_steering_decode(tmp.steering);
				fb_ptr->from_radio.front_pitch_sp = cmmc_radio_control_front_pitch_decode(tmp.front_pitch);
				fb_ptr->from_radio.front_roll_sp = cmmc_radio_control_front_roll_decode(tmp.front_roll);
				fb_ptr->from_radio.rear_pitch_sp = cmmc_radio_control_rear_pitch_decode(tmp.rear_pitch);

				fb_ptr->from_radio.arm_switch = cmmc_radio_control_arm_switch_decode(tmp.arm_switch);
				fb_ptr->from_radio.mode_switch = cmmc_radio_control_mode_switch_decode(tmp.mode_switch);

				g_can_messages_decoded++;
				break;

			default:
				break;
			}
			g_can_messages_arrived++;
		}
		task_can_rx_alive++;
		osThreadYield();	//<- preffered no delay inside this task loop in freeRTOS
	}
}


