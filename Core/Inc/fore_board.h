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
	/* UBX-NAV-PVT BEGIN */
    // UTC time
    uint16_t year;
    uint8_t  month, day;
    uint8_t  hour, min, sec;

    // Validity flag: bit0=validDate; bit1=validTime; bit2=fullyResolved; bit3=validMag
    uint8_t  valid;
    uint32_t tAcc_ns;  // Time accuracy estimate (UTC)
    int32_t  nano_ns;  // Fraction of second, range -1e9 .. 1e9 (UTC)

	// GNSSfix Type: 0 = no fix; 1 = dead reckoning only;
    // 2 = 2D-fix; 3 = 3D-fix; 4 = GNSS + dead reckoning combined; 5 = time only fix
    uint8_t fixType;

    uint8_t flags;
    uint8_t flags2;

    // Number of satellites used in Nav Solution
    uint8_t numSV;
    int32_t lon, lat; 	// Longitude and Latitude scaled by 1e-7

    int32_t height_mm;	// Height above ellipsoid
	int32_t hMSL_mm; 	// Height above mean sea level

	uint32_t hAcc_mm; 	// Horizontal accuracy estimate
	uint32_t vAcc_mm; 	// Vertical accuracy estimate

	int32_t  gSpeed_mms; 	// Ground Speed (2-D)
	int32_t  headMot_deg1e5; 	// Heading of motion (2-D)
	uint32_t sAcc_mms;		// Speed accuracy estimate
	uint32_t headAcc_deg1e5;  // Heading accuracy estimate (both motion and vehicle)

	uint16_t pDOP_centi; // Position DOP multiplied by 100

	uint16_t flags3;
    /* UBX-NAV-PVT END */

	/* UBX-MON-VER */
	char swVersion[30];	// Nul-terminated software version string.
	char hwVersion[10]; // Nul-terminated hardware version string
	uint8_t n_times; // Number of additional 30 char nul-terminated strings that can be read. Determined from message length 40 + [0..n]·30

} gnss_info_t;

typedef struct {
	servo_power_data_t left_servo_power;
	servo_power_data_t right_servo_power;

	servo_feedback_t left_servo_feedback;
	servo_feedback_t right_servo_feedback;

	range_meas_t left_tof;
	range_meas_t right_tof;

	radio_controls_t from_radio;

	auto_control_t from_controller;

	gnss_info_t gnss_info;
} fore_board_t;

void fore_board_init(void);

fore_board_t* fore_board_get_ptr(void);

#endif /* INC_FORE_BOARD_H_ */
