#pragma once
#include <AnalogMux.h>

namespace Analog
{


	uint16_t ADC_Req(uint8_t address)
	{
		return 123;
	}

	void Result(uint8_t address, uint16_t value)
	{

	}

	AnalogMux<0> obj
	(
		ADC_Req, Result/*,
		EasyPinD::d_pin_t{GPIOA, GPIO_PIN_2}, 
		EasyPinD::d_pin_t{GPIOA, GPIO_PIN_2}, 
		EasyPinD::d_pin_t{GPIOA, GPIO_PIN_2}*/
	);



};
