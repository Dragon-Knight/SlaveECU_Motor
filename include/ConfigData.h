#pragma once
#include <inttypes.h>

namespace Config
{
	struct __attribute__((packed)) eeprom_body_t
	{
		uint16_t wheel_diameter = 620;		// Диаметр колеса, мм.

		struct
		{
			uint16_t min = 150;
			uint16_t max = 950;
		} pwm;
	};
};