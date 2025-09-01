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
    ACTUATORS_OFF,
    ACTUATORS_ON,
    ACTUATORS_AUTO_CALIBRATION,
	ACTUATORS_RANGE_IDENTIFICATION,
} actuator_state_t;

typedef struct {
	TIM_HandleTypeDef *handle_ptr;
	uint32_t channel;
	uint32_t setpoint;
} timer_wrapper_t;

typedef struct {
	actuator_state_t 	prev_state;
	actuator_state_t 	state;

	timer_wrapper_t 	left_servo;
	timer_wrapper_t 	right_servo;
} servo_actuators_t;


servo_actuators_t actuators = {
		.prev_state = ACTUATORS_OFF,
		.state = ACTUATORS_OFF, // ACTUATORS_IDENTIFICATION
		.left_servo = {
			.handle_ptr = &htim1,
			.channel = 	TIM_CHANNEL_1,
		},
		.right_servo = {
			.handle_ptr = &htim1,
			.channel = 	TIM_CHANNEL_2,
		},
};


// FT1117M 120* for range 900 to 2100 that gives 120/(2100-900) = 120 / 1200 = 1/10
// 1 degree per 10 us
// mechanism range is +12/-6 deg which is 18 degrees
// The upper arm is 18.8 the lower arm is 25 mm
// So the (25/18,8) * 18 = 23.936 is the movement of the upper arm?
// from onshape 17,343 + 6,643 = 23.986 degrees of servo motion
// so 24 degrees of motion is like 240 us of control

static inline void set_left_servo_sp(servo_actuators_t* hact, uint32_t sp)
{
	hact->left_servo.setpoint = sp;
	__HAL_TIM_SET_COMPARE(hact->left_servo.handle_ptr, hact->left_servo.channel, sp);
}

static inline void set_right_servo_sp(servo_actuators_t* hact, uint32_t sp)
{
	hact->right_servo.setpoint = sp;
	__HAL_TIM_SET_COMPARE(hact->right_servo.handle_ptr, hact->right_servo.channel, sp);
}

static inline void start_servo(servo_actuators_t* hact)
{
	HAL_TIM_PWM_Start(hact->left_servo.handle_ptr, hact->left_servo.channel);
	HAL_TIM_PWM_Start(hact->right_servo.handle_ptr, hact->right_servo.channel);
}

static inline void stop_servo(servo_actuators_t* hact)
{
	HAL_TIM_PWM_Stop(hact->left_servo.handle_ptr, hact->left_servo.channel);
	HAL_TIM_PWM_Stop(hact->right_servo.handle_ptr, hact->right_servo.channel);
}

static uint32_t actuators_range_identification(void);

extern volatile uint32_t task_servo_control_alive;
void task_servo_control(void* argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();


	while (1)
	{
		/* Obtain control */
//		fb_ptr->
// todo control state change via fb_ptr

		/* Execute control */
		switch (actuators.state)
		{
		case ACTUATORS_OFF:
			if (actuators.prev_state != actuators.state)
			{
				stop_servo(&actuators);
			}
			break;

		case ACTUATORS_ON:
			if (actuators.prev_state != actuators.state)
			{
				start_servo(&actuators);
			}
// todo control ia fb_ptr

			break;

		case ACTUATORS_AUTO_CALIBRATION:
			if (actuators.prev_state != actuators.state)
			{
				start_servo(&actuators);
			}

			static uint32_t left = 1500;
			static uint32_t right = 1500;

			set_left_servo_sp(&actuators, left);
			set_right_servo_sp(&actuators, right);
			break;

		case ACTUATORS_RANGE_IDENTIFICATION:
			if (actuators.prev_state != actuators.state)
			{
				start_servo(&actuators);
			}

			uint32_t sp = actuators_range_identification();

			set_left_servo_sp(&actuators, sp);
			set_right_servo_sp(&actuators, sp);
			break;

		default:
			if (actuators.prev_state != actuators.state)
			{
				stop_servo(&actuators);
			}
			break;
		}
		actuators.prev_state = actuators.state;

		task_servo_control_alive++;
		osDelay(10);
	}
}


static uint32_t actuators_range_identification(void)
{
	static enum {
		IDLE_AT_900,
		STEP_TO_2100,
		IDLE_AT_2100,
		SWEEP_UP,
		IDLE_AT_TOP,
		SWEEP_DOWN
	} id_state = IDLE_AT_900;

	static uint32_t calib_sp = 900;
	static uint32_t hold_counter = 0;
	static uint32_t sweep_counter = 0;

	switch(id_state)
	{
	case IDLE_AT_900:
		calib_sp = 900;
		if (++hold_counter >= 100) // 100 * 10ms = 1s
		{
			hold_counter = 0;
			id_state = STEP_TO_2100;
		}
		break;

	case STEP_TO_2100:
		calib_sp = 2100;
		id_state = IDLE_AT_2100;
		break;

	case IDLE_AT_2100:
		if (++hold_counter >= 100)
		{
			hold_counter = 0;
			id_state = SWEEP_UP;  // go to sweep up (incrementing)
			calib_sp = 900;       // start sweep from 900
		}
		break;

	case SWEEP_UP:
		if (++sweep_counter >= 10) // N iterations between increments
		{
			sweep_counter = 0;
			if (calib_sp < 2100)
				calib_sp++;
			else
				id_state = IDLE_AT_TOP;
		}
		break;

	case IDLE_AT_TOP:
		if (++hold_counter >= 100)
		{
			hold_counter = 0;
			id_state = SWEEP_DOWN;
		}
		break;

	case SWEEP_DOWN:
		if (++sweep_counter >= 10)
		{
			sweep_counter = 0;
			if (calib_sp > 900)
				calib_sp--;
			else
				id_state = IDLE_AT_900;
		}
		break;
	}
	return calib_sp;
}
