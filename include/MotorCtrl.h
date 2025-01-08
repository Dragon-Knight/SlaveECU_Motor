#pragma once
#include <inttypes.h>
#include <EasyPinA.h>
//#include "MovingAverage.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

namespace MotorCtrl
{
	
	enum gear_t : uint8_t
	{
		GEAR_NEUTRAL = 0b00000000,
		GEAR_FORWARD_LOW = 0b00000001,
		GEAR_FORWARD_HI = 0b00000010,
		GEAR_REVERSE = 0b00000100,
		GEAR_MASK = 0b00000111
	};
	static uint8_t BREAK_RECOVERY_BIT = 4;
	static uint8_t LOCK_BIT = 7;

	static constexpr uint16_t PWM_MIN = 150;
	static constexpr uint16_t PWM_MAX = 950;


	
	EasyPinA throttle(&hadc1, GPIOA, GPIO_PIN_1, ADC_CHANNEL_1, ADC_SAMPLETIME_55CYCLES_5);
	//MovingAverage<uint16_t, uint32_t, 12> throttle_val;


	
	// Управление передачей
	void SetGear(uint8_t idx, gear_t gear)
	{
		SPI::hc595.WriteByMask(idx, gear, GEAR_MASK);
		
		return;
	}
	
	// Управление рекупирацией (тормозом)
	void SetBreak(uint8_t idx, bool state)
	{
		SPI::hc595.SetState(idx, BREAK_RECOVERY_BIT, state);
		
		return;
	}

	// Управление питанием контроллера (замок зажигания)
	void SetLock(uint8_t idx, bool state)
	{
		SPI::hc595.SetState(idx, LOCK_BIT, state);
		
		return;
	}





	bool CheckThrottleValue(uint16_t value)
	{
		return (value > 200 && value < 3850);
	}








	void HardwareSetup()
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_MIN);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_MIN);
		
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		
		return;
	}
	
	inline void Setup()
	{
		HardwareSetup();

		throttle.Init();

		//SetGear(0, GEAR_REVERSE);


		CANLib::obj_throttle_value_1.RegisterFunctionSetRealtime
		(
			// Колбек realtime данных
			[](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				uint16_t throttle = (can_frame.data[1] | (can_frame.data[2] << 8));

				DEBUG_LOG_TOPIC("ThrlVal", "motor: 1, idx: %d, val: %d\n", can_frame.data[0], throttle);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, throttle);

				return CAN_RESULT_IGNORE;
			}, 
			// Колбек нарушения логики приёма
			[](uint32_t time_has_passed_ms) -> void
			{
				DEBUG_LOG_TOPIC("ThrlVal", "motor: 1, ERROR\n");
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				
				// Защита отключена.
				CANLib::obj_throttle_value_1.ResetRealtimeErrorState();
			}, 
			// Настройки
			100, 0, true, 3
		);
		CANLib::obj_throttle_value_1.ResetRealtimeErrorState();

		CANLib::obj_throttle_value_2.RegisterFunctionSetRealtime
		(
			// Колбек realtime данных
			[](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				uint16_t throttle = (can_frame.data[1] | (can_frame.data[2] << 8));

				DEBUG_LOG_TOPIC("ThrlVal", "motor: 2, idx: %d, val: %d\n", can_frame.data[0], throttle);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, throttle);

				return CAN_RESULT_IGNORE;
			}, 
			// Колбек нарушения логики приёма
			[](uint32_t time_has_passed_ms) -> void
			{
				DEBUG_LOG_TOPIC("ThrlVal", "motor: 2, ERROR\n");
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

				// Защита отключена.
				CANLib::obj_throttle_value_2.ResetRealtimeErrorState();
			}, 
			// Настройки
			100, 0, true, 3
		);
		CANLib::obj_throttle_value_2.ResetRealtimeErrorState();
		
		CANLib::obj_transmission_value_1.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			SetGear(0, (gear_t)can_frame.data[0]);
			
			return CAN_RESULT_IGNORE;
		});
		CANLib::obj_transmission_value_2.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			SetGear(1, (gear_t)can_frame.data[0]);
			
			return CAN_RESULT_IGNORE;
		});

		CANLib::obj_brakerecuperation_flag_1.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			bool state = (can_frame.data[0] == 0) ? false : true;
			SetBreak(0, state);
			
			return CAN_RESULT_IGNORE;
		});
		CANLib::obj_brakerecuperation_flag_2.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			bool state = (can_frame.data[0] == 0) ? false : true;
			SetBreak(1, state);
			
			return CAN_RESULT_IGNORE;
		});
		
		CANLib::obj_ignitionlock_flag_1.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			bool state = (can_frame.data[0] == 0) ? false : true;
			SetLock(0, state);
			
			return CAN_RESULT_IGNORE;
		});
		CANLib::obj_ignitionlock_flag_2.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			bool state = (can_frame.data[0] == 0) ? false : true;
			SetLock(1, state);
			
			return CAN_RESULT_IGNORE;
		});
		

		return;
	}

	uint16_t val1 = 0;

	const uint32_t gist1 = 10;
	const uint32_t gist2 = 50;
	const uint32_t gist3 = 100;
	const uint32_t sample_counter = 5;
	uint32_t counter_pos = 0;
	uint32_t counter_neg = 0;
	uint32_t adc_val = 0;
	
	inline void Loop(uint32_t &current_time)
	{

		static uint32_t tick_20 = 0;
		if(current_time - tick_20 > 20)
		{
			tick_20 = current_time;

			
			uint16_t throttle_adc = throttle.Get();
			throttle_adc >>= 2;
			
			if( throttle_adc > adc_val )
			{
				counter_pos++;
				counter_neg = 0;
			}
			else if( throttle_adc < adc_val )
			{
				counter_neg++;
				counter_pos = 0;
			}

			if( throttle_adc > (adc_val + gist3) )
			{
				if( counter_pos >= sample_counter )
					adc_val += gist3;
			}
			else if( throttle_adc < (adc_val - gist3) )
			{
				if( counter_neg >= sample_counter )
					adc_val -= gist3;
			}

			if( throttle_adc > (adc_val + gist2) && throttle_adc <= (adc_val + gist3) )
			{
				if( counter_pos >= sample_counter )
					adc_val += gist2;
			}
			else if( throttle_adc < (adc_val - gist2) && throttle_adc >= (adc_val - gist3) )
			{
				if( counter_neg >= sample_counter )
					adc_val -= gist2;
			}

			if( throttle_adc > (adc_val + gist1) )
			{
				if( counter_pos >= sample_counter )
					adc_val++;
			}
			else if( throttle_adc < (adc_val - gist1) )
			{
				if( counter_neg >= sample_counter )
					adc_val--;
			}




/*
			//throttle_val.Push(throttle_adc);
			DEBUG_LOG_TOPIC("Motor", "throttle: %04d\n", throttle_adc);
			Logger.Printf(">Motor raw:%d\n", throttle_adc);
			Logger.Printf(">Motor calc:%d\n", adc_val);
			if(CheckThrottleValue(throttle_adc) == true)
			{

			}
*/
/*			
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, val1);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (1023 - val1));
			if(++val1 == 1023) val1 = 0;
*/			
			



		}
		
		
		
		current_time = HAL_GetTick();
		
		return;
	}
}
