#pragma once
#include <inttypes.h>
#include <EasyPinA.h>

extern ADC_HandleTypeDef hadc1;

namespace MotorCtrl
{
	
	enum gear_t : uint8_t
	{
		GEAR_NEUTRAL = 0b00000000,
		GEAR_FORWARD_LOW = 0b00000001,
		GEAR_FORWARD_HI = 0b00000010,
		GEAR_REVERSE = 0b00000100
	};
	static uint8_t BREAK_RECOVERY = 0b00010000;
	static uint8_t LOCK = 0b10000000;


	
	EasyPinA throttle(&hadc1, GPIOA, GPIO_PIN_1, ADC_CHANNEL_1);



	
	// Управление передачей
	void SetGear(uint8_t idx, gear_t gear)
	{
		SPI::hc595.WriteByMask(idx, gear, 0b00000111);
		
		return;
	}
	
	// Управление рекупирацией (тормозом)
	void SetBreak(uint8_t idx, bool state)
	{
		SPI::hc595.SetState(idx, 4, state);
		
		return;
	}

	// Управление питанием контроллера (замок зажигания)
	void SetLock(uint8_t idx, bool state)
	{
		SPI::hc595.SetState(idx, 7, state);
		
		return;
	}





	bool CheckThrottleValue(uint16_t value)
	{
		return (value > 200 && value < 3850);
	}










	inline void Setup()
	{
		throttle.Init();
		throttle.Calibration();
		

		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{

		static uint32_t tick_20 = 0;
		if(current_time - tick_20 > 20)
		{
			tick_20 = current_time;
			
			uint16_t throttle_adc = throttle.Get();
			DEBUG_LOG_TOPIC("Motor", "throttle: %04d\n", throttle_adc);
			if(CheckThrottleValue(throttle_adc) == true)
			{

			}
		}
		
		
		
		current_time = HAL_GetTick();
		
		return;
	}
}
