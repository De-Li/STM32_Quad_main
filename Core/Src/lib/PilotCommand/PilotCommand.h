#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../Math/Math.h"
#include "../EJ_namespace.h"
#include "stm32f4xx_hal.h"

//SBUS
#define SBUSBUF_SIZE 25 //The buffer size for HAL uart receive.
#define RXBUF_SIZE 50 //MAX buff for parsing SBUS signal.
#define PARSED_SBUS_SIZE 14

/* define SBUS range mapping here, -+100% -> 1220..2060*/
#define SBUS_RANGE_MIN 200
#define SBUS_RANGE_MAX 1800
#define SBUS_RANGE_RANGE (SBUS_RANGE_MAX - SBUS_RANGE_MIN)

/*#define SBUS_TARGET_MIN 1220
#define SBUS_TARGET_MAX 2060
#define SBUS_TARGET_RANGE (SBUS_TARGET_MAX - SBUS_TARGET_MIN)

#define SBUS_SCALE_OFFSET (SBUS_TARGET_MIN - ((SBUS_TARGET_RANGE * SBUS_RANGE_MIN / SBUS_RANGE_RANGE)))*/

//RC Input parameter
#define ROLL_RATE_MAX 90 * DEG_TO_RAD
#define PITCH_RATE_MAX 90 * DEG_TO_RAD
#define YAW_RATE_MAX 60 * DEG_TO_RAD
#define ROLL_ANGLE_MAX 30 * DEG_TO_RAD
#define PITCH_ANGLE_MAX 30 * DEG_TO_RAD

class PilotCommand{
public:
	PilotCommand();
//	{
//		start_time = HAL_GetTick();
//		SBUS_received_count = 0;
//
//		//HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));
//	}
	void init();

	void read_rc_input();

	uint16_t* get_rc();

	bool emergency_event();

	void update_flight_mode();

	void get_pilot_command_rate(float &roll_rate_out, float &pitch_rate_out);

	void get_pilot_command_angle(float &roll_angle_out, float &pitch_angle_out);

	float get_pilot_yaw_rate();

	float get_pilot_throttle();

	bool has_new_input();

	uint8_t rx_buffer[SBUSBUF_SIZE];
	uint8_t temp_buffer[RXBUF_SIZE];

	enum class mode : uint8_t {
		acro = 0,
		stabilize = 1,
	};
	mode flightmode;

private:
	void parse_sbus();

	void decode_channels(uint16_t*);

	uint32_t start_time;
	uint8_t main_buffer[RXBUF_SIZE];
	uint8_t sbus_data[SBUSBUF_SIZE];
	uint16_t rc_input[PARSED_SBUS_SIZE];
	uint8_t previous_stop_byte = 0x00;
	uint32_t SBUS_received_count;

	//RC Input parameter
	int16_t RC_ROLL_MIN = 1220;
	int16_t RC_ROLL_MID;
	int16_t RC_ROLL_RANGE;
	int16_t RC_ROLL_MAX = 2060;

	int16_t RC_PITCH_MIN = 1220;
	int16_t RC_PITCH_MID;
	int16_t RC_PITCH_RANGE;
	int16_t RC_PITCH_MAX = 2060;

	int16_t RC_THROTTLE_MIN = 1221;
	int16_t RC_THROTTLE_MID;
	int16_t RC_THROTTLE_RANGE;
	int16_t RC_THROTTLE_MAX = 2058;

	int16_t RC_YAW_MIN = 1220;
	int16_t RC_YAW_MID;
	int16_t RC_YAW_RANGE;
	int16_t RC_YAW_MAX = 2060;
};
