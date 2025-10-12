/*
 * task_can_tx.c
 *
 *  Created on: Oct 12, 2025
 *      Author: brzan
 */
#include "task_can_rx.h"
#include "main.h"
#include "cmsis_os2.h"
#include "fore_board.h"

#include "can-not/can_not.h"
#include "can-messages-mini-celka/src/cmmc.h"

static void send_distance_fore_feedback(fore_board_t* fb_ptr)
{
	struct cmmc_distance_fore_feedback_t tmp = {
			.range_mm_l = cmmc_distance_achter_feedback_range_mm_l_encode(fb_ptr->left_tof.range_mm),
			.signal_rate_mcps_l = fb_ptr->left_tof.signalRate_mcps,	// do not use encode here
			.error_status_l = cmmc_distance_achter_feedback_error_status_l_encode(fb_ptr->left_tof.errorStatus),
			.range_mm_r = cmmc_distance_achter_feedback_range_mm_r_encode(fb_ptr->right_tof.range_mm),
			.signal_rate_mcps_r = fb_ptr->right_tof.signalRate_mcps, // do not use encode here
			.error_status_r = cmmc_distance_achter_feedback_error_status_r_encode(fb_ptr->right_tof.errorStatus),
	};
	cant_generic_struct_t msg = {
			.msg_dlc	= CMMC_DISTANCE_ACHTER_FEEDBACK_LENGTH,
			.msg_id		= CMMC_DISTANCE_ACHTER_FEEDBACK_FRAME_ID,
			.msg_payload = { 0U },
	};
	cmmc_distance_fore_feedback_pack(msg.msg_payload, &tmp, msg.msg_dlc);
	cant_transmit(&msg);
}

static void send_actuator_left_foil_feedback(fore_board_t* fb_ptr)
{
	struct cmmc_actuator_left_foil_feedback_t tmp = {
			.current = cmmc_actuator_left_foil_feedback_current_encode(fb_ptr->left_servo_power.current),
			.voltage = cmmc_actuator_left_foil_feedback_voltage_encode(fb_ptr->left_servo_power.voltage),
			.power = cmmc_actuator_left_foil_feedback_power_encode(fb_ptr->left_servo_power.power),
			.setpoint_us = cmmc_actuator_left_foil_feedback_setpoint_us_encode(fb_ptr->left_servo_feedback.setpoint_us),
			.position_raw = cmmc_actuator_left_foil_feedback_position_raw_encode(fb_ptr->left_servo_feedback.raw_adc),
			.mode = cmmc_actuator_left_foil_feedback_mode_encode(fb_ptr->left_servo_feedback.mode),
	};
	cant_generic_struct_t msg = {
			.msg_dlc	= CMMC_ACTUATOR_LEFT_FOIL_FEEDBACK_LENGTH,
			.msg_id		= CMMC_ACTUATOR_LEFT_FOIL_FEEDBACK_FRAME_ID,
			.msg_payload = { 0U },
	};
	cmmc_actuator_left_foil_feedback_pack(msg.msg_payload, &tmp, msg.msg_dlc);
	cant_transmit(&msg);
}

static void send_actuator_right_foil_feedback(fore_board_t* fb_ptr)
{
	struct cmmc_actuator_right_foil_feedback_t tmp = {
			.current = cmmc_actuator_right_foil_feedback_current_encode(fb_ptr->right_servo_power.current),
			.voltage = cmmc_actuator_right_foil_feedback_voltage_encode(fb_ptr->right_servo_power.voltage),
			.power = cmmc_actuator_right_foil_feedback_power_encode(fb_ptr->right_servo_power.power),
			.setpoint_us = cmmc_actuator_right_foil_feedback_setpoint_us_encode(fb_ptr->right_servo_feedback.setpoint_us),
			.position_raw = cmmc_actuator_right_foil_feedback_position_raw_encode(fb_ptr->right_servo_feedback.raw_adc),
			.mode = cmmc_actuator_right_foil_feedback_mode_encode(fb_ptr->right_servo_feedback.mode),
	};
	cant_generic_struct_t msg = {
			.msg_dlc	= CMMC_ACTUATOR_RIGHT_FOIL_FEEDBACK_LENGTH,
			.msg_id		= CMMC_ACTUATOR_RIGHT_FOIL_FEEDBACK_FRAME_ID,
			.msg_payload = { 0U },
	};
	cmmc_actuator_right_foil_feedback_pack(msg.msg_payload, &tmp, msg.msg_dlc);
	cant_transmit(&msg);
}

static uint32_t distance_fore_feedback_message_period = 10; // centy seconds (10cs is 100ms)
static uint32_t actuator_left_foil_feedback_message_period = 10;
static uint32_t actuator_right_foil_feedback_message_period = 10;

extern volatile uint32_t task_can_tx_alive;
void task_can_tx(void *argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();

	uint32_t distance_fore_feedback_message_counter = 0;
	uint32_t actuator_left_foil_feedback_message_counter = 0;
	uint32_t actuator_right_foil_feedback_message_counter = 0;

	while (1)
	{
		if (++distance_fore_feedback_message_counter >= distance_fore_feedback_message_period)
		{
			distance_fore_feedback_message_counter = 0;
			send_distance_fore_feedback(fb_ptr);
		}

		if (++actuator_left_foil_feedback_message_counter >= actuator_left_foil_feedback_message_period)
		{
			actuator_left_foil_feedback_message_counter = 0;
			send_actuator_left_foil_feedback(fb_ptr);
		}

		if (++actuator_right_foil_feedback_message_counter >= actuator_right_foil_feedback_message_period)
		{
			actuator_right_foil_feedback_message_counter = 0;
			send_actuator_right_foil_feedback(fb_ptr);
		}

		task_can_tx_alive++;
		osDelay(10);
	}
}
