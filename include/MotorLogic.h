#pragma once
#include <stm32f1xx_hal.h>
#include <MotorManager.h>
#include <drivers/MotorFardriverOld.h>
#include <drivers/MotorFardriverNew.h>


extern UART_HandleTypeDef hMotor1Uart;
extern UART_HandleTypeDef hMotor2Uart;

/*
void OnMotorEvent(const uint8_t motor_idx, motor_packet_raw_t *raw_packet);
void OnMotorError(const uint8_t motor_idx, const motor_error_t code);
void OnMotorHWError(const uint8_t motor_idx, const uint8_t code);
void OnMotorTX(const uint8_t motor_idx, const uint8_t *raw, const uint8_t raw_len);
*/
namespace Motors
{
	void MOTOR_UART_TX(uint8_t idx, const uint8_t *raw, const uint16_t length);
	void MOTOR_ERROR(uint8_t idx, int8_t code);

#warning Check packet req connection from controller. Check data
	
	
	MotorFardriverNew motor1(123456);
	MotorFardriverOld motor2(123456);
	MotorManager manager(MOTOR_UART_TX, MOTOR_ERROR);
	
	struct data_t
	{
		UART_HandleTypeDef *hal;		// Указатель на объект HAL USART
		uint8_t hot[200];				// Горячий массив данных (Работа в прерывании)
	} uart_data[2];
	
	enum motor_num_t : uint8_t { MOTOR_1 = 0, MOTOR_2 = 1 };
	
	
	inline void UART_RX(uint8_t idx, const uint16_t length)
	{
		manager.RawRx(idx, uart_data[idx].hot, length);
		
		return;
	}
	
	void MOTOR_UART_TX(uint8_t idx, const uint8_t *raw, const uint16_t length)
	{
		HAL_UART_Transmit(uart_data[idx].hal, (uint8_t *)raw, length, 64U);
		
		return;
	}
	
	void MOTOR_ERROR(uint8_t idx, int8_t code)
	{
		DEBUG_LOG_TOPIC("MOTOR-ERR", "idx: %d, code: %d\n", idx, code);
	}
	
	
	inline void Setup()
    {
		memset(uart_data, 0x00, sizeof(uart_data));
		
		uart_data[MOTOR_1].hal = &hMotor1Uart;
		uart_data[MOTOR_2].hal = &hMotor2Uart;

/*
		motor1.SetReadyCallback([](const FdNew::packet_raw_t *data)
		{
			if(data->_A1 != 0xB6) return;

			// Заполяем данные, но при этом частота обновлений будет примерно 1.5с
		});
*/	

		manager.SetModel(MOTOR_1, motor1);
		manager.SetModel(MOTOR_2, motor2);
		
		HAL_UARTEx_ReceiveToIdle_IT(uart_data[MOTOR_1].hal, uart_data[MOTOR_1].hot, sizeof(uart_data[MOTOR_1].hot));
		HAL_UARTEx_ReceiveToIdle_IT(uart_data[MOTOR_2].hal, uart_data[MOTOR_2].hot, sizeof(uart_data[MOTOR_2].hot));
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		manager.Tick(current_time);

		static uint32_t tick_25 = 0;
		if(current_time - tick_25 > 25UL)
		{
			tick_25 = current_time;
			
			MotorManagerData::common_data_t *common_data;
			for(uint8_t idx = 0; idx < 2; ++idx)
			{
				if( manager.common_data_ready[idx] == false ) continue;
				
				common_data = &manager.common_data[idx];
				/*
				CANLib::obj_controller_rpm.SetValue(idx, common_data->rpm, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_controller_speed.SetValue(idx, common_data->speed, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_controller_gear_n_roll.SetValue(2 * idx, common_data->gear, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_controller_gear_n_roll.SetValue(2 * idx + 1, common_data->roll, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_controller_voltage.SetValue(idx, common_data->voltage, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_controller_current.SetValue(idx, common_data->current, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_controller_power.SetValue(idx, common_data->power, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_controller_temperature.SetValue(idx, common_data->temp_controller, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_motor_temperature.SetValue(idx, common_data->temp_motor, CAN_TIMER_TYPE_NORMAL);
				if(common_data->errors > 0)
				{
					CANLib::obj_controller_errors.SetValue(idx, common_data->errors, CAN_TIMER_TYPE_CRITICAL, CAN_EVENT_TYPE_NORMAL);
				}*/
			}
		}

		current_time = HAL_GetTick();
		
		return;
	}
}
