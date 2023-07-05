/*
	Класс парсинга пакетов UART контроллеров FarDriver:
		Nanjing FarDriver Controller Co., Ltd;
		SIAYQ FarDriver Controller;
	Отдельное спасибо Китайцам, которые зажопили протокол...

	PS: Для упрощения работы с пакетом, данные принимаются в обратном порядке, т.е. порядок байт такой:
		[ CRC | D11 | D10 | D9 | D8 | D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0 | A1 | A0 ]
*/

#pragma once

#include "MotorErrors.h"
#include "MotorPackets.h"

using event_callback_t = void (*)(const uint8_t motor_idx, motor_packet_raw_t *packet);
using error_callback_t = void (*)(const uint8_t motor_idx, const motor_error_t code);
using tx_callback_t = void (*)(const uint8_t motor_idx, const uint8_t *raw, const uint8_t raw_len);

/*************************************************************************************
 *
 * FardriverControllerInterface: It is used for pointer operations with controllers.
 *
 *************************************************************************************/
class FardriverControllerInterface
{
public:
	virtual void SetEventCallback(event_callback_t callback) = 0;
	virtual void SetErrorCallback(error_callback_t callback) = 0;
	virtual void SetTXCallback(tx_callback_t callback) = 0;
	virtual void RXByte(uint8_t data, uint32_t time) = 0;
	virtual bool IsActive() = 0;
	virtual void Processing(uint32_t time) = 0;
};

/*************************************************************************************
 *
 * FardriverController: implements FardriverControllerInterface.
 *
 *************************************************************************************/
template <uint8_t _motor_idx = 0>
class FardriverController : public FardriverControllerInterface
{
	static const uint16_t _rx_buffer_timeout = 10; // Время мс до сброса принимаемого пакета.
	static const uint8_t _rx_buffer_size = 16;	   // Общий размер пакета.
	static const uint16_t _request_time = 550;	   // Интервал отправки запраса данных в контроллер.
	static const uint16_t _unactive_timeout = 500; // Время мс бездейтсвия, после которого считается что связи с контроллером нет.

public:
	FardriverController()
	{
		_ClearBuff();
	}

	/*
		Регистрирует колбек, который возвращает принятый пакет.
	*/
	virtual void SetEventCallback(event_callback_t callback) override
	{
		_event_callback = callback;
	}

	/*
		Регистрирует колбек, который возвращает принятый флаг(и) ошибок.
	*/
	virtual void SetErrorCallback(error_callback_t callback) override
	{
		_error_callback = callback;
	}

	/*
		Регистрирует колбек, который отправляет данные контроллеру.
	*/
	virtual void SetTXCallback(tx_callback_t callback) override
	{
		_tx_callback = callback;
	}

	/*
		(Interrupt) Вставка принятого байта и обработка пакета.
	*/
	virtual void RXByte(uint8_t data, uint32_t time) override
	{
		// Если с момента последнего байта прошло более _packet_timeout мс.
		if (time - _rx_buffer_timeout > _rx_buffer_last_time)
		{
			_ClearBuff();
		}

		// Если есть место для нового байта.
		if (_rx_buffer_idx > 0)
		{
			_rx_buffer_last_time = time;
			_rx_buffer[--_rx_buffer_idx] = data;
		}

		// Если приняли весь пакет
		if (_rx_buffer_idx == 0)
		{
			_ValidateBuffer();
		}
	}

	/*
		Флаг активного соединенеия с контроллером.
	*/
	virtual bool IsActive() override
	{
		return _isActive;
	}

	/*
		Обработка принытых данных.
		Вызываться должна с интервалом, не более 30 мс!
	*/
	virtual void Processing(uint32_t time) override
	{
		// Нужно ответить на запрос авторизации.
		if (_need_init_tx == true)
		{
			_tx_callback(_motor_idx, motor_packet_init_tx, sizeof(motor_packet_init_tx));
			_need_init_tx = false;
		}

		// Каждые _request_time отправляет запросы в контроллер.
		if (time - _request_last_time > _request_time)
		{
			_request_last_time = time;
			_tx_callback(_motor_idx, motor_packet_request_tx, sizeof(motor_packet_request_tx));
		}

		// Обработка принятого пакета.
		if (_work_buffer_ready == true)
		{
			// Вычитываем ошибки, и вызываем колбек, если нужно.
			if (_error_callback != nullptr)
			{
				motor_packet_0_t *obj = (motor_packet_0_t *)_work_buffer;

				if (obj->_A1 == 0x00 && obj->ErrorFlags != _lastErrorFlags)
				{
					_lastErrorFlags = obj->ErrorFlags;
					_error_callback(_motor_idx, obj->ErrorFlags);
				}
			}

			// Вызываем колбек события, если он зарегистрирован.
			if (_event_callback != nullptr)
			{
				motor_packet_raw_t *obj = (motor_packet_raw_t *)_work_buffer;

				_event_callback(_motor_idx, obj);
			}

			_work_buffer_ready = false;
		}

		// Время послденего байта больше _unactive_timeout.
		if (time - _rx_buffer_last_time > _unactive_timeout)
		{
			_isActive = false;
		}
	}

	static int16_t TempFix(uint8_t raw_temp)
	{
		return (raw_temp <= 200) ? (uint8_t)raw_temp : (int8_t)raw_temp;
	}

private:
	/*
		Проверяет принятый пакет на валидность.
	*/
	inline void _ValidateBuffer()
	{
		// Если приняли 'нормальный' пакет.
		if (_rx_buffer[_rx_buffer_size - 1] == 0xAA)
		{
			motor_packet_raw_t *obj = (motor_packet_raw_t *)_rx_buffer;
			if (_GetBuffCRC() == obj->_CRC)
			{
				memcpy(&_work_buffer, _rx_buffer, _rx_buffer_size);
				_work_buffer_ready = true;
				_isActive = true;
			}
		}
		// Если приняли пакет авторизации.
		else if (memcmp(&motor_packet_init_rx, &_rx_buffer, _rx_buffer_size) == 0)
		{
			_need_init_tx = true;
		}
	}

	/*
		Расчитывает CRC принятого пакета.
	*/
	inline uint16_t _GetBuffCRC()
	{
		uint16_t result = 0x0000;

		for (uint8_t i = 2; i < _rx_buffer_size; ++i)
		{
			result += _rx_buffer[i];
		}

		return result;
	}

	/*
		Очищает буфер.
	*/
	void _ClearBuff()
	{
		memset(&_rx_buffer, 0x00, _rx_buffer_size);
		_rx_buffer_idx = _rx_buffer_size;
	}

	event_callback_t _event_callback = nullptr;
	error_callback_t _error_callback = nullptr;
	tx_callback_t _tx_callback = nullptr;

	uint8_t _rx_buffer[_rx_buffer_size]; // Приёмный (горячий) буфер.
	uint8_t _rx_buffer_idx;				 // Индекс принимаего байта.
	uint32_t _rx_buffer_last_time = 0;	 // Время мс последнего принятого байта.

	uint8_t _work_buffer[_rx_buffer_size]; // Рабочий (холодный) буфер.
	bool _work_buffer_ready = false;	   // Готовность рабочего буфера.

	uint16_t _lastErrorFlags = 0x0000;

	bool _need_init_tx = false;

	uint32_t _request_last_time = 0;

	bool _isActive;

	// enum state_t : uint8_t {STATE_IDLE, STATE_AUTH, STATE_WORK, STATE_LOST} _state = STATE_IDLE;
};
