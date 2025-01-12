#pragma once
#include <inttypes.h>

namespace FardriverNew
{
	// Пакет которым флудит контроллер. Запрос на подключение
	static constexpr uint8_t PacketInitRx[] = {0x41, 0x54, 0x2B, 0x56, 0x45, 0x52, 0x53, 0x49, 0x4F, 0x4E};
	
	// Пакет, которым нужно ответить, чтобы контроллер установил связь
	static constexpr uint8_t PacketInitTx[] = {0xAA, 0x13, 0xEC, 0x07, 0x01, 0xF1, 0xA2, 0x5D};
	
	// Размер пакета принимаемых данных
	static constexpr uint8_t PacketSize = 16U;
	
	// Заголовок пакета
	static constexpr uint8_t PacketHeader[] = {0xAA};
	
	// Пакет запроса данных с контроллера
	static constexpr uint8_t PacketRequest[] = {0xAA, 0x13, 0xEC, 0x07, 0x01, 0xF1, 0xA2, 0x5D};
	
	// Интервал запроса данных из контроллера
	static constexpr uint16_t PacketRequestInerval = 1250;
	
	
	// Типы ошибок контроллера
	typedef enum : uint16_t
	{
		MOTOR_ENCODER_ALARM = (1U << 0),
		ACCELERATION_PEDAL_ALARM = (1U << 1),
		CURRENT_PROTECT_RESTART = (1U << 2),
		PHASE_CURRENT_MUTATION = (1U << 3),
		VOLTAGE_ALARM = (1U << 4),
		BURGLAR_ALARM = (1U << 5),
		MOTOR_OVER_TEMP = (1U << 6),
		CONTROLLER_OVER_TEMP = (1U << 7),
		PHASE_CURRENT_OVERFLOW = (1U << 8),
		PHASE_ZERO_ALARM = (1U << 9),
		PHASE_SHORT_ALARM = (1U << 10),
		LINE_CURR_ZERO_ALARM = (1U << 11),
		MOSFET_HIGHSIDE_BRIDGE_ALARM = (1U << 12),
		MOSFET_LOWSIDE_BRIDGE_ALARM = (1U << 13),
		MOE_CURRENT_ALARM = (1U << 14),
		BRAKE_ALARM = (1U << 15)
	} error_t;
	
	// 
	struct __attribute__((__packed__)) packet_raw_t
	{
		uint8_t _A0;
		uint8_t _A1;
		uint8_t _D0;
		uint8_t _D1;
		uint8_t _D2;
		uint8_t _D3;
		uint8_t _D4;
		uint8_t _D5;
		uint8_t _D6;
		uint8_t _D7;
		uint8_t _D8;
		uint8_t _D9;
		uint8_t _D10;
		uint8_t _D11;
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_raw_t) == PacketSize, "Structures should have the same size!");
	
	// 
	struct __attribute__((__packed__)) packet_80_t
	{
		uint8_t _A0;
		uint8_t _A1;
		uint8_t gear : 4;			// A1: 0x80, D0, bitmask
		uint8_t roll : 4;			// A1: 0x80, D0, bitmask
		uint8_t _D1;
		error_t error_flags;		// A1: 0x80, D2-3, bitmask
		uint8_t _D4;
		uint8_t _D5;
		uint16_t rpm;				// A1: 0x80, D6-7, x1
		uint8_t _D8;
		uint8_t _D9;
		uint8_t _D10;
		uint8_t _D11;
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_80_t) == PacketSize, "Structures should have the same size!");
	
	// 
	struct __attribute__((__packed__)) packet_81_t
	{
		uint8_t _A0;
		uint8_t _A1;
		uint16_t voltage;			// A1: 0x81, D0-1, x0.1
		uint8_t _D2;
		uint8_t _D3;
		int16_t current;			// A1: 0x81, D4-5, x0.25
		uint8_t _D6;
		uint8_t _D7;
		uint8_t _D8;
		uint8_t _D9;
		int16_t throttle;			// A1: 0x81, D10-11, x1
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_81_t) == PacketSize, "Structures should have the same size!");
	
	// 
	struct __attribute__((__packed__)) packet_B3_t
	{
		uint8_t _A0;
		uint8_t _A1;
		uint8_t _D0;
		uint8_t _D1;
		uint8_t _D2;
		uint8_t _D3;
		uint8_t _D4;
		uint8_t _D5;
		uint8_t _D6;
		uint8_t _D7;
		uint8_t _D8;
		uint8_t _D9;
		int16_t temp_controller;	// A1: 0xB3, D10-11, x1
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_B3_t) == PacketSize, "Structures should have the same size!");
	
	// 
	struct __attribute__((__packed__)) packet_B5_t
	{
		uint8_t _A0;
		uint8_t _A1;
		int16_t temp_motor;			// A1: 0xB5, D0-1, x1
		uint8_t _D2;
		uint8_t _D3;
		uint8_t _D4;
		uint8_t _D5;
		uint8_t _D6;
		uint8_t _D7;
		uint8_t _D8;
		uint8_t _D9;
		uint8_t _D10;
		uint8_t _D11;
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_B5_t) == PacketSize, "Structures should have the same size!");
	
}
