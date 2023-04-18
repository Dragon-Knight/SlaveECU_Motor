#include <Arduino.h>
#include "FardriverController.h"

FardriverController<1> motor1;
FardriverController<2> motor2;

void OnMotorEvent(uint8_t num, uint8_t data[16])
{

}

void OnMotorError(uint8_t num, motor_error_t code)
{

}

void setup()
{

	motor1.SetEventCallback(OnMotorEvent);
	motor2.SetEventCallback(OnMotorEvent);
	motor1.SetErrorCallback(OnMotorError);
	motor2.SetErrorCallback(OnMotorError);
}


void loop()
{
	static uint32_t current_time = millis();
	
	motor1.Processing(current_time);
	motor2.Processing(current_time);

	if(Serial.available() > 0)
	{
		motor1.RXByte( Serial.read() , current_time);
		motor2.RXByte( Serial.read() , current_time);
	}

}
