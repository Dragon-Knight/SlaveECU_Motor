#pragma once
#include <inttypes.h>

// 
enum gear_t : uint8_t
{
	GEAR_NEUTRAL = 0x00,
	GEAR_FORWARD = 0x01,
	GEAR_REVERSE = 0x02,
	GEAR_LOW = 0x03,
	GEAR_BOOST = 0x04,
	GEAR_UNKNOWN = 0x0F
};

// 
enum roll_t : uint8_t
{
	ROLL_STOP = 0x00,
	ROLL_FORWARD = 0x01,
	ROLL_REVERSE = 0x02,
	ROLL_UNKNOWN = 0x0F
};

struct motor_common_data_t
{
	uint16_t errors;
	uint16_t rpm;				// 1.0 RPM
	gear_t gear;
	roll_t roll;
	uint16_t voltage;			// 0.1 V
	int16_t current;			// 0.1 A
	int16_t throttle;			// 1.0 %
	int16_t temp_controller;	// 1.0 C
	int16_t temp_motor;			// 1.0 C
	uint32_t odometer;
	uint16_t speed;				// 0.1 kmh
	int16_t power;				// 1.0 W
};
