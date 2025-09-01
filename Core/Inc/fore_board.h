/*
 * fore_board.h
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */

#ifndef INC_FORE_BOARD_H_
#define INC_FORE_BOARD_H_

typedef struct {
	float angle;	// This is obtained via CAN interface
} control_request_t;

typedef struct {
	float voltage;
} servo_feedback_t;

typedef struct {
    float voltage;
    float current;
} servo_power_data_t;

typedef struct {
	servo_power_data_t left_servo_power;
	servo_power_data_t right_servo_power;

	servo_feedback_t left_servo_feedback;
	servo_feedback_t right_servo_feedback;
} fore_board_t;

void fore_board_init(void);

fore_board_t* fore_board_get_ptr(void);

#endif /* INC_FORE_BOARD_H_ */
