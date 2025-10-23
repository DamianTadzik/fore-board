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
#include "math.h"

typedef enum {
    ACTUATOR_OFF,
    ACTUATOR_ON,
	ACTUATOR_AUTO_RANGE_IDENTIFICATION,
	ACTUATOR_DEVELOPMENT,
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

	int16_t		calibrated_setpoint_lower_bound;
	int16_t		calibrated_setpoint_upper_bound;
} actuator_t;


actuator_t right_foil_actuator = {
		.prev_state = ACTUATOR_OFF,
		.state = ACTUATOR_OFF,
		.timer = {
				.handle_ptr = &htim1,
				.channel = 	TIM_CHANNEL_2,
				.setpoint_lower_bound = 1000,
				.setpoint_upper_bound = 2000,
		},
		.calibrated_setpoint_lower_bound = 1326,
		.calibrated_setpoint_upper_bound = 1608,
};
actuator_t left_foil_actuator = {
		.prev_state = ACTUATOR_OFF,
		.state = ACTUATOR_OFF,
		.timer = {
				.handle_ptr = &htim1,
				.channel = 	TIM_CHANNEL_1,
				.setpoint_lower_bound = 1000,
				.setpoint_upper_bound = 2000,
		},
		.calibrated_setpoint_lower_bound = 1434,
		.calibrated_setpoint_upper_bound = 1691,
};

// FT1117M 120* for range 900 to 2100 that gives 120/(2100-900) = 120 / 1200 = 1/10
// 1 degree per 10 us
// mechanism range is +12/-6 deg which is 18 degrees
// The upper arm is 18.8 the lower arm is 25 mm
// So the (25/18,8) * 18 = 23.936 is the movement of the upper arm?
// from onshape 17,343 + 6,643 = 23.986 degrees of servo motion
// so 24 degrees of motion is like 240 us of control signal

// so 1 degree of foil movement is like 25/18.8 = 1.33 degrees of servo movement which is 13us
// other way around, 1us is

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

typedef enum {
	HOMING_IDLE,
	HOMING_MOVE_TO_LOWER,
	HOMING_VERIFY_LOWER,
	HOMING_MOVE_TO_UPPER,
	HOMING_VERIFY_UPPER,
	HOMING_RETURN_TO_CENTER,
	HOMING_DONE,
	HOMING_ERROR
} homing_state_t;

typedef struct {
	homing_state_t phase;
	uint16_t setpoint;
	uint32_t stable_counter;
	uint32_t timeout_start;
	uint32_t next_step_time;
	float filtered_current;
	int16_t lower_found;
	int16_t upper_found;
} homing_ctx_t;

static homing_ctx_t right_homing_ctx = {0};
static homing_ctx_t left_homing_ctx  = {0};

const static float homing_filter_alpha = 0.1f;      // current low-pass filter
const static uint16_t homing_step_us = 1;           // PWM step per loop
const static uint32_t homing_step_interval_ms = 50; // time between PWM updates
const static uint32_t homing_loop_period_ms = 10;   // same as osDelay(10)
const static uint32_t homing_verify_time_ms = 200;  // time to confirm stall
const static uint32_t homing_timeout_ms = 30000;     // total safety timeout

static int actuator_range_identification(actuator_t *hact, float *ptr_current)
{
	homing_ctx_t *ctx = (hact == &right_foil_actuator) ? &right_homing_ctx : &left_homing_ctx;

	uint32_t now;

	// Low-pass filter on current
	ctx->filtered_current = (1.0f - homing_filter_alpha) * ctx->filtered_current +
	                        homing_filter_alpha * fabs((*ptr_current));

	switch (ctx->phase)
	{
	case HOMING_IDLE:
		ctx->setpoint = 1500;  // start from safe middle
		actuator_set_setpoint(hact, ctx->setpoint);

		ctx->lower_found = 0;
		ctx->upper_found = 0;

		ctx->timeout_start = osKernelGetTickCount();
		ctx->next_step_time = ctx->timeout_start + 500;  // wait 0.5 s

		ctx->phase = HOMING_MOVE_TO_LOWER;
		break;

	case HOMING_MOVE_TO_LOWER:
	    now = osKernelGetTickCount();
	    if (now < ctx->next_step_time)
	        break;
	    ctx->next_step_time = now + homing_step_interval_ms;
		// if there is room for another move
		if (ctx->setpoint > 1000 + homing_step_us)
		{
			// if current is still under the threshold
			if (ctx->filtered_current < 0.08f) // [A]
			{
				// Move towards the endpoint
				ctx->setpoint -= homing_step_us;
				// Clear stable counter
				ctx->stable_counter = 0;
			}
			else
			{
				ctx->stable_counter += homing_loop_period_ms;
				// If we are long enough over the limit
				if (ctx->stable_counter >= homing_verify_time_ms)
				{
					ctx->stable_counter = 0;

					ctx->lower_found = ctx->setpoint;
					ctx->phase = HOMING_VERIFY_LOWER;
					break;
				}
			}
		}
		else
		{
			ctx->phase = HOMING_ERROR;
			break;
		}
		actuator_set_setpoint(hact, ctx->setpoint);

		// Timeout protection
		if (osKernelGetTickCount() - ctx->timeout_start > homing_timeout_ms)
			ctx->phase = HOMING_ERROR;
		break;

	case HOMING_VERIFY_LOWER:
		// Move away a bit to release tension
	    now = osKernelGetTickCount();
	    if (now < ctx->next_step_time)
	        break;
	    ctx->next_step_time = now + homing_step_interval_ms;
		// If there is room to move
		if (ctx->setpoint < 2000 - homing_step_us)
		{
			// if current is still above the threshold
			if (ctx->filtered_current > 0.025f) // [A]
			{
				// Move away from the endpoint
				ctx->setpoint += homing_step_us;
			}
			else
			{
				ctx->lower_found = ctx->setpoint;
				ctx->phase = HOMING_MOVE_TO_UPPER;
				break;
			}
		}
		else
		{
			ctx->phase = HOMING_ERROR;
			break;
		}
		actuator_set_setpoint(hact, ctx->setpoint);

		// Timeout protection
		if (osKernelGetTickCount() - ctx->timeout_start > homing_timeout_ms)
			ctx->phase = HOMING_ERROR;
		break;


	case HOMING_MOVE_TO_UPPER:
	    now = osKernelGetTickCount();
	    if (now < ctx->next_step_time)
	        break;
	    ctx->next_step_time = now + homing_step_interval_ms;
		// if there is room for another move
		if (ctx->setpoint < 2000 - homing_step_us)
		{
			// if current is still under the threshold
			if (ctx->filtered_current < 0.08f) // [A]
			{
				// Move towards the endpoint
				ctx->setpoint += homing_step_us;
				// Clear stable counter
				ctx->stable_counter = 0;
			}
			else
			{
				ctx->stable_counter += homing_loop_period_ms;
				// If we are long enough over the limit
				if (ctx->stable_counter >= homing_verify_time_ms)
				{
					ctx->stable_counter = 0;

					ctx->upper_found = ctx->setpoint;
					ctx->phase = HOMING_VERIFY_UPPER;
					break;
				}
			}
		}
		else
		{
			ctx->phase = HOMING_ERROR;
			break;
		}
		actuator_set_setpoint(hact, ctx->setpoint);

		// Timeout protection
		if (osKernelGetTickCount() - ctx->timeout_start > homing_timeout_ms)
			ctx->phase = HOMING_ERROR;
		break;

	case HOMING_VERIFY_UPPER:
		// Move away a bit to release tension
	    now = osKernelGetTickCount();
	    if (now < ctx->next_step_time)
	        break;
	    ctx->next_step_time = now + homing_step_interval_ms;
		// If there is room to move
		if (ctx->setpoint > 1000 + homing_step_us)
		{
			// if current is still above the threshold
			if (ctx->filtered_current > 0.025f) // [A]
			{
				// Move away from the endpoint
				ctx->setpoint -= homing_step_us;
			}
			else
			{
				ctx->upper_found = ctx->setpoint;
				ctx->phase = HOMING_RETURN_TO_CENTER;
				break;
			}
		}
		else
		{
			ctx->phase = HOMING_ERROR;
			break;
		}
		actuator_set_setpoint(hact, ctx->setpoint);

		// Timeout protection
		if (osKernelGetTickCount() - ctx->timeout_start > homing_timeout_ms)
			ctx->phase = HOMING_ERROR;
		break;


	case HOMING_RETURN_TO_CENTER:
		ctx->setpoint = (ctx->lower_found + ctx->upper_found) / 2;
		actuator_set_setpoint(hact, ctx->setpoint);
		ctx->phase = HOMING_DONE;
		break;

	case HOMING_DONE:
		// Done save edge values in the scalling register
		hact->calibrated_setpoint_lower_bound = ctx->lower_found+2;
		hact->calibrated_setpoint_upper_bound = ctx->upper_found-2;
		return 1;

	case HOMING_ERROR:
	default:
		actuator_disable(hact);
		return -1;
	}

	return 0; // not done yet
}

volatile int16_t development_setpoint = 1500;

static void actuator_update_state(actuator_t *hact,
								  int16_t setpoint_us,
								  servo_feedback_t *servo_feedback,
								  servo_power_data_t *servo_power)
{
	switch (hact->state)
	{
	case ACTUATOR_OFF:
		if (hact->prev_state != hact->state)
			actuator_disable(hact);
		break;

	case ACTUATOR_ON:
		if (hact->prev_state != hact->state)
			actuator_enable(hact);
		actuator_set_setpoint(hact, setpoint_us);
		break;

	case ACTUATOR_DEVELOPMENT:
		if (hact->prev_state != hact->state)
			actuator_enable(hact);
		actuator_set_setpoint(hact, development_setpoint);
		break;

	case ACTUATOR_AUTO_RANGE_IDENTIFICATION:
		if (hact->prev_state != hact->state)
			actuator_enable(hact);
		if (actuator_range_identification(hact, &servo_power->current))
			hact->state = ACTUATOR_DEVELOPMENT;
		break;

	default:
		if (hact->prev_state != hact->state)
			actuator_disable(hact);
		break;
	}

	// Update feedback
	servo_feedback->mode = hact->state;
	servo_feedback->setpoint_us = __HAL_TIM_GET_COMPARE(hact->timer.handle_ptr, hact->timer.channel);

	// Update previous state
	hact->prev_state = hact->state;
}

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

		// Wzmocnienia
		int16_t gain_pitch = 1000;
		int16_t gain_roll  = 1000;

		// left = pitch + roll
		// right = pitch - roll
		int16_t left_cmd  = (gain_pitch * pitch + gain_roll * roll) / 1000;
		int16_t right_cmd = (gain_pitch * pitch - gain_roll * roll) / 1000;

		// saturacja
		if (left_cmd > 1000) left_cmd = 1000;
		else if (left_cmd < -1000) left_cmd = -1000;
		if (right_cmd > 1000) right_cmd = 1000;
		else if (right_cmd < -1000) right_cmd = -1000;

		// skalowanie
		int16_t left_sp = map_i16(left_cmd, -1000, 1000,
				left_foil_actuator.calibrated_setpoint_lower_bound,
				left_foil_actuator.calibrated_setpoint_upper_bound);
		int16_t right_sp = map_i16(-right_cmd, -1000, 1000,
				right_foil_actuator.calibrated_setpoint_lower_bound,
				right_foil_actuator.calibrated_setpoint_upper_bound);

//		/* Setpoint source selection based on the radio switch */ TO BE DONE
//		if (fb_ptr->from_radio.mode_switch == MODE_RC_CONTROL)
//		{
//
//		}
//		else if (fb_ptr->from_radio.mode_switch == MODE_RC_CONTROL)
//		{
//
//		}
//		else if (fb_ptr->from_radio.mode_switch == MODE_RC_CONTROL)
//		{
//
//		}

		/* Obtain the control from the radio */
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
		else if (fb_ptr->from_radio.arm_switch == 0x04)
		{
			left_foil_actuator.state = ACTUATOR_DEVELOPMENT;
			right_foil_actuator.state = ACTUATOR_DEVELOPMENT;
		}
		else
		{
			left_foil_actuator.state = ACTUATOR_AUTO_RANGE_IDENTIFICATION;
			right_foil_actuator.state = ACTUATOR_AUTO_RANGE_IDENTIFICATION;
		}

		/* Execute control alghoritm of left foil actuator */
		actuator_update_state(&left_foil_actuator, left_sp,
							  &fb_ptr->left_servo_feedback,
							  &fb_ptr->left_servo_power);

		/* Execute control alghoritm of right foil actuator */
		actuator_update_state(&right_foil_actuator, right_sp,
							  &fb_ptr->right_servo_feedback,
							  &fb_ptr->right_servo_power);

		task_servo_control_alive++;
		osDelay(10);
	}
}
