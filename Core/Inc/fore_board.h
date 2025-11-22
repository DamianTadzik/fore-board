/*
 * fore_board.h
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */

#ifndef INC_FORE_BOARD_H_
#define INC_FORE_BOARD_H_

#include "main.h"

typedef enum {
	DISARMED,
	ARMED_STEERING_PROPULSION,
	ARMED_ALL,
} arm_switch_t;

typedef enum {
	MANUAL_RADIO,
	MANUAL_CONTROLLER,
	AUTO_CONTROLLER,
} mode_switch_t;

typedef struct {
	int16_t throttle;
	int16_t steering;

	int16_t front_pitch_sp;
	int16_t front_roll_sp;
	int16_t rear_pitch_sp;

	arm_switch_t arm_switch;
	mode_switch_t mode_switch;
} radio_controls_t;

typedef struct {
	float front_left_angle_sp;
	float front_right_angle_sp;
	float rear_angle_sp;
} auto_control_t;

typedef struct {
	float voltage;
	uint16_t raw_adc;
	int16_t setpoint_us;
	uint8_t mode;
} servo_feedback_t;

typedef struct {
	uint8_t range_mm;
	int32_t signalRate_mcps;
	uint8_t errorStatus;
} range_meas_t;

typedef struct {
    float voltage;
    float current;
    float power;
} servo_power_data_t;

typedef struct {
	servo_power_data_t left_servo_power;
	servo_power_data_t right_servo_power;

	servo_feedback_t left_servo_feedback;
	servo_feedback_t right_servo_feedback;

	range_meas_t left_tof;
	range_meas_t right_tof;

	radio_controls_t from_radio;

	auto_control_t from_controller;
} fore_board_t;

void fore_board_init(void);

fore_board_t* fore_board_get_ptr(void);

#endif /* INC_FORE_BOARD_H_ */
