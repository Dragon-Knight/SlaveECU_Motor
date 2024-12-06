#pragma once
#include <LEDLibrary.h>

namespace Leds
{
	enum leds_t : uint8_t
	{
		LED_NONE = 0,
		LED_WHITE = 1,
		LED_GREEN = 2,
		LED_RED = 3
	};
	
	InfoLeds<3> obj;
	
	inline void Setup()
	{
		obj.AddLed( {GPIOC, GPIO_PIN_13}, LED_WHITE );
		obj.AddLed( {GPIOC, GPIO_PIN_14}, LED_GREEN );
		obj.AddLed( {GPIOC, GPIO_PIN_15}, LED_RED );
		
		
		obj.SetOn(LED_WHITE);
		HAL_Delay(100);
		obj.SetOff(LED_WHITE);
		
		obj.SetOn(LED_GREEN);
		HAL_Delay(100);
		obj.SetOff(LED_GREEN);
		
		obj.SetOn(LED_RED);
		HAL_Delay(100);
		obj.SetOff(LED_RED);
		
		Leds::obj.SetOn(Leds::LED_GREEN, 50, 1950);
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		obj.Processing(current_time);

		current_time = HAL_GetTick();

		return;
	}
}
