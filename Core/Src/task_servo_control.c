/*
 * task_servo_control.c
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */
#include "task_servo_control.h"
#include "main.h"
#include "cmsis_os2.h"
#include "fore_board.h"

#include "tim.h"

typedef enum {
    ACTUATOR_OFF,
    ACTUATOR_ON,
    ACTUATOR_AUTO_CALIBRATION,
	ACTUATOR_RANGE_IDENTIFICATION,
} actuator_state_t;

typedef struct {
	TIM_HandleTypeDef *handle_ptr;
	uint32_t channel;
	int16_t requested_setpoint;
	int16_t setpoint_lower_bound;
	int16_t setpoint_upper_bound;
} timer_wrapper_t;

typedef struct {
	actuator_state_t 	prev_state;
	actuator_state_t 	state;
	timer_wrapper_t 	timer;
} actuator_t;


actuator_t right_foil_actuator = {
		.prev_state = ACTUATOR_OFF,
		.state = ACTUATOR_OFF, // ACTUATORS_IDENTIFICATION
		.timer = {
				.handle_ptr = &htim1,
				.channel = 	TIM_CHANNEL_2,
				.setpoint_lower_bound = 1000,
				.setpoint_upper_bound = 2000,
		},
};
actuator_t left_foil_actuator = {
		.prev_state = ACTUATOR_OFF,
		.state = ACTUATOR_OFF, // ACTUATORS_IDENTIFICATION
		.timer = {
				.handle_ptr = &htim1,
				.channel = 	TIM_CHANNEL_1,
				.setpoint_lower_bound = 1000,
				.setpoint_upper_bound = 2000,
		},
};

// FT1117M 120* for range 900 to 2100 that gives 120/(2100-900) = 120 / 1200 = 1/10
// 1 degree per 10 us
// mechanism range is +12/-6 deg which is 18 degrees
// The upper arm is 18.8 the lower arm is 25 mm
// So the (25/18,8) * 18 = 23.936 is the movement of the upper arm?
// from onshape 17,343 + 6,643 = 23.986 degrees of servo motion
// so 24 degrees of motion is like 240 us of control

static inline int16_t clamp_i16(int16_t x, int16_t lb, int16_t ub)
{
    if (x > ub) return ub;
    if (x < lb) return lb;
    return x;
}

static inline int16_t map_i16(int16_t x,
                              int16_t in_min,  int16_t in_max,
                              int16_t out_min, int16_t out_max)
{
    if (in_max == in_min) return out_min;
    int32_t num   = (int32_t)(x - in_min) * (out_max - out_min);
    int32_t denom = (int32_t)(in_max - in_min);
    return (int16_t)(out_min + num / denom);
}


static void actuator_set_setpoint(actuator_t *hact, int16_t requested_setpoint)
{
	hact->timer.requested_setpoint = requested_setpoint;
	int16_t clamped_setpoint = clamp_i16(requested_setpoint, hact->timer.setpoint_lower_bound, hact->timer.setpoint_upper_bound);
	__HAL_TIM_SET_COMPARE(hact->timer.handle_ptr, hact->timer.channel, (uint32_t)clamped_setpoint);
}

static void actuator_enable(actuator_t *hact)
{
//	HAL_TIM_PWM_Start(hact->timer.handle_ptr, hact->timer.channel);
    if (IS_TIM_BREAK_INSTANCE(hact->timer.handle_ptr->Instance))
    {
        __HAL_TIM_MOE_ENABLE(hact->timer.handle_ptr);
    }
    // włącz sam kanał (CCxE)
    TIM_CCxChannelCmd(hact->timer.handle_ptr->Instance,
                      hact->timer.channel,
                      TIM_CCx_ENABLE);
}

static void actuator_disable(actuator_t *hact)
{
//	HAL_TIM_PWM_Stop(hact->timer.handle_ptr, hact->timer.channel);
    // wyłącz sam kanał (CCxE)
    TIM_CCxChannelCmd(hact->timer.handle_ptr->Instance,
                      hact->timer.channel,
                      TIM_CCx_DISABLE);
    if (IS_TIM_BREAK_INSTANCE(hact->timer.handle_ptr->Instance))
    {
        __HAL_TIM_MOE_DISABLE(hact->timer.handle_ptr);
    }
}

static uint32_t actuators_range_identification(void);

extern volatile uint32_t task_servo_control_alive;
void task_servo_control(void* argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();

	while (1)
	{
		/* Obtain control signals from CAN/RADIO */
		// Joystick inputs (int16_t) in range [-1000, 1000]
		int16_t roll  = fb_ptr->from_radio.front_roll_sp;
		int16_t pitch = fb_ptr->from_radio.front_pitch_sp;

		// Wzmocnienia (skala w promilach dla łatwego tuningu, 1000 = 1.0)
		int16_t gain_pitch = 1000;
		int16_t gain_roll  = 1000;

		// left = pitch + roll
		// right = pitch - roll
		// wynik w tysiącach => dzielimy przez 1000 aby wrócić do skali [-1000,1000]
		int16_t left_cmd  = (gain_pitch * pitch + gain_roll * roll) / 1000;
		int16_t right_cmd = (gain_pitch * pitch - gain_roll * roll) / 1000;

		// saturacja
		if (left_cmd > 1000) left_cmd = 1000;
		else if (left_cmd < -1000) left_cmd = -1000;
		if (right_cmd > 1000) right_cmd = 1000;
		else if (right_cmd < -1000) right_cmd = -1000;

		// bazowanie

		int16_t left_sp = map_i16(left_cmd, -1000, 1000, 1440, 1680);
		int16_t right_sp = map_i16(-right_cmd, -1000, 1000, 1330, 1610);

		if (fb_ptr->from_radio.arm_switch == ARMED_ALL)
		{
			left_foil_actuator.state = ACTUATOR_ON;
			right_foil_actuator.state = ACTUATOR_ON;
		}
		else if (fb_ptr->from_radio.arm_switch == ARMED_STEERING_PROPULSION || fb_ptr->from_radio.arm_switch == DISARMED)
		{
			left_foil_actuator.state = ACTUATOR_OFF;
			right_foil_actuator.state = ACTUATOR_OFF;
		}

		/* Execute control alghoritm of left foil actuator */
		switch (left_foil_actuator.state)
		{
		case ACTUATOR_OFF:
			if (left_foil_actuator.prev_state != left_foil_actuator.state)
				actuator_disable(&left_foil_actuator);
			break;
		case ACTUATOR_ON:
			if (left_foil_actuator.prev_state != left_foil_actuator.state)
				actuator_enable(&left_foil_actuator);
			actuator_set_setpoint(&left_foil_actuator, left_sp);
			break;
		default:
			if (left_foil_actuator.prev_state != left_foil_actuator.state)
				actuator_disable(&left_foil_actuator);
			break;
		}
		left_foil_actuator.prev_state = left_foil_actuator.state;

		/* Execute control alghoritm of right foil actuator */
		switch (right_foil_actuator.state)
		{
		case ACTUATOR_OFF:
			if (right_foil_actuator.prev_state != right_foil_actuator.state)
				actuator_disable(&right_foil_actuator);
			break;
		case ACTUATOR_ON:
			if (right_foil_actuator.prev_state != right_foil_actuator.state)
				actuator_enable(&right_foil_actuator);
			actuator_set_setpoint(&right_foil_actuator, right_sp);
			break;
		default:
			if (right_foil_actuator.prev_state != right_foil_actuator.state)
				actuator_disable(&right_foil_actuator);
			break;
		}
		right_foil_actuator.prev_state = right_foil_actuator.state;

		/* Save setpoint and mode in the fb structure for feedback */
		fb_ptr->left_servo_feedback.mode = left_foil_actuator.state;
		fb_ptr->left_servo_feedback.setpoint_us = left_sp;

		fb_ptr->right_servo_feedback.mode = right_foil_actuator.state;
		fb_ptr->right_servo_feedback.setpoint_us = right_sp;

		task_servo_control_alive++;
		osDelay(10);
	}
}


//static uint32_t actuators_range_identification(void)
//{
//	static enum {
//		IDLE_AT_900,
//		STEP_TO_2100,
//		IDLE_AT_2100,
//		SWEEP_UP,
//		IDLE_AT_TOP,
//		SWEEP_DOWN
//	} id_state = IDLE_AT_900;
//
//	static uint32_t calib_sp = 900;
//	static uint32_t hold_counter = 0;
//	static uint32_t sweep_counter = 0;
//
//	switch(id_state)
//	{
//	case IDLE_AT_900:
//		calib_sp = 900;
//		if (++hold_counter >= 100) // 100 * 10ms = 1s
//		{
//			hold_counter = 0;
//			id_state = STEP_TO_2100;
//		}
//		break;
//
//	case STEP_TO_2100:
//		calib_sp = 2100;
//		id_state = IDLE_AT_2100;
//		break;
//
//	case IDLE_AT_2100:
//		if (++hold_counter >= 100)
//		{
//			hold_counter = 0;
//			id_state = SWEEP_UP;  // go to sweep up (incrementing)
//			calib_sp = 900;       // start sweep from 900
//		}
//		break;
//
//	case SWEEP_UP:
//		if (++sweep_counter >= 10) // N iterations between increments
//		{
//			sweep_counter = 0;
//			if (calib_sp < 2100)
//				calib_sp++;
//			else
//				id_state = IDLE_AT_TOP;
//		}
//		break;
//
//	case IDLE_AT_TOP:
//		if (++hold_counter >= 100)
//		{
//			hold_counter = 0;
//			id_state = SWEEP_DOWN;
//		}
//		break;
//
//	case SWEEP_DOWN:
//		if (++sweep_counter >= 10)
//		{
//			sweep_counter = 0;
//			if (calib_sp > 900)
//				calib_sp--;
//			else
//				id_state = IDLE_AT_900;
//		}
//		break;
//	}
//	return calib_sp;
//}
