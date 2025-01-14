#pragma once
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <MotorManager.h>
#include <MotorDeviceInterface.h>
#include "MotorFardriverOld_Data.h"

namespace FdOld = FardriverOld;
namespace MMD = MotorManagerData;

class MotorFardriverOld : public MotorDeviceInterface
{
	using callback_ready_t = void (*)(const FdOld::packet_raw_t *data);
	
	public:
		
		MotorFardriverOld() : MotorDeviceInterface(), _callback_ready(nullptr), 
			_last_request_time(0), _last_response_time(0), _auth(false), _need_auth(false), 
			_busy(false), _ready(false)
		{
			memset(_data, 0x00, sizeof(_data));
			
			return;
		}
		
		void SetReadyCallback(callback_ready_t ready)
		{
			_callback_ready = ready;
			
			return;
		}
		
		virtual void Init() override
		{
			return;
		}
		
		virtual void Tick(uint32_t &time) override
		{
			if(_need_auth == true)
			{
				_manager->RawTx(_idx, FdOld::PacketInitTx, sizeof(FdOld::PacketInitTx));
				
				_need_auth = false;
			}
			
			// Если данные от контроллера приняты
			if(_ready == true)
			{
				_ready = false;
			
				// Т.к. данные представленны в формате big-endian, обращаем массив для просторы работы
				_ReverseArray(_data, sizeof(_data));

				_PostProcessing();

				// Напихиваем полезные данные в общий объект
				FdOld::packet_raw_t *raw = (FdOld::packet_raw_t *) _data;
				MMD::common_data_t *common_data = &_manager->common_data[_idx];
				
				switch(raw->_A1)
				{
					case 0x00:
					{
						FdOld::packet_00_t *packet = (FdOld::packet_00_t *) _data;
						
						if(common_data->errors != packet->error_flags)
						{
							_error.code = (packet->error_flags > 0) ? ERROR_CTRL : ERROR_NONE;
						}
						
						common_data->errors = packet->error_flags;
						common_data->rpm = packet->rpm >> 2;
						common_data->speed = ((_speed_coefficient * packet->rpm) / 400000UL);
						common_data->gear = FixGear(packet->gear);
						common_data->roll = FixRoll(packet->roll);
						
						break;
					}
					case 0x01:
					{
						FdOld::packet_01_t *packet = (FdOld::packet_01_t *) _data;
						
						int16_t power = (((uint32_t)abs(packet->current) * (uint32_t)packet->voltage) / 40U);
						if(packet->current < 0) power = -power;
						
						common_data->voltage = packet->voltage;
						common_data->current = ((packet->current * 10U) / 4U);
						common_data->power = power;
						common_data->throttle = packet->throttle;
						
						break;
					}
					case 0x04:
					{
						FdOld::packet_04_t *packet = (FdOld::packet_04_t *) _data;
						
						common_data->temp_controller = FixTemp(packet->temp_controller);
						
						break;
					}
					case 0x0D:
					{
						FdOld::packet_0D_t *packet = (FdOld::packet_0D_t *) _data;
						
						common_data->temp_motor = FixTemp(packet->temp_motor);
						
						break;
					}
					case 0x0C:
					{
						// Пока не получим первый раз последний пакет в посылки от контроллера (0x0C) - считаем что данные не готовы.
						// На самом деле это предпоследний пакет посылки, но последний это 0x00 и его сложно отследить.
						// В последствии считаем что данные актуальны, хоть и опаздывают.
						_manager->common_data_ready[_idx] = true;
						
						break;
					}
				}

				_last_response_time = time;
				
				if(_callback_ready != nullptr)
				{
					_callback_ready(raw);
				}
				
				_busy = false;
			}
			
			// Если данные от контроллера не приняты
			else
			{
				// Если от контроллера давно не было данных
				if(_error.code != ERROR_LOST && time - _last_response_time > 1000)
				{
					_manager->ResetCommonData(_idx);
					_error.code = ERROR_LOST;
				}
			}

			if(time - _last_request_time > FdOld::PacketRequestInerval /*&& _auth == true*/)
			{
				_last_request_time = time;

				_manager->RawTx(_idx, FdOld::PacketRequest, sizeof(FdOld::PacketRequest));
			}
			
			return;
		}
		
		// Приём пакета, в прерывании. Минимум самый важный действий.
		virtual void RawRx(const uint8_t *raw, const uint8_t length) override
		{
			_error.code = ERROR_NONE;
			
			if(memcmp(FdOld::PacketInitRx, raw, sizeof(FdOld::PacketInitRx)) == 0)
			{
				_need_auth = true; return;
			}
			if(_busy == true){ _error.code = ERROR_BUSY; return; }
			if(length != FdOld::PacketSize){ _error.code = ERROR_LENGTH; return; }
			if(memcmp(FdOld::PacketHeader, raw, sizeof(FdOld::PacketHeader)) != 0){ _error.code = ERROR_HEADER; return; }
			if(_CheckCRCSum(raw) == false){ _error.code = ERROR_CRC; return; }
			
			memcpy(_data, raw, sizeof(_data));
			
			_ready = true;
			_busy = true;
			_auth = true;
			
			return;
		}
		
		static inline int16_t FixTemp(uint8_t raw)
		{
			return (raw <= 200) ? (uint8_t)raw : (int8_t)raw;
		}
		
		static inline MMD::gear_t FixGear(uint8_t raw)
		{
			// 0С - 00 - Нейтраль
			// 01 - 01 - Передняя
			// 0E - 02 - Задняя
			
			MMD::gear_t gear;
			switch(raw)
			{
				case 0x0C: { gear = MMD::GEAR_NEUTRAL; break; }
				case 0x01: { gear = MMD::GEAR_FORWARD; break; }
				case 0x0E: { gear = MMD::GEAR_REVERSE; break; }
				default:   { gear = MMD::GEAR_UNKNOWN; break; }
			}
			
			return gear;
		}
		
		static inline MMD::roll_t FixRoll(uint8_t raw)
		{
			// 00 - Стоп
			// 01 - Назад
			// 03 - Вперёд
			
			MMD::roll_t roll;
			switch(raw)
			{
				case 0x00: { roll = MMD::ROLL_STOP; break; }
				case 0x01: { roll = MMD::ROLL_REVERSE; break; }
				case 0x03: { roll = MMD::ROLL_FORWARD; break; }
				default:   { roll = MMD::ROLL_UNKNOWN; break; }
			}
			
			return roll;
		}
		
	private:
		
		bool _CheckCRCSum(const uint8_t *array)
		{
			uint16_t crc = 0x0000;
			
			for(uint8_t i = 0; i < 14; ++i) { crc += array[i]; }
			
			return ( ((crc >> 8) & 0xFF) == array[14] && (crc & 0xFF) == array[15] );
		}
		
		void _PostProcessing()
		{
			return;
		}

		callback_ready_t _callback_ready;	// Колбек принятого пакета, если указан
		uint32_t _last_request_time;		// Время последнего цикла обновления данных
		uint32_t _last_response_time;		// Время последнего принятого пакета данных
		bool _auth;							// Флаг пройденной авторизации
		bool _need_auth;					// Флаг необходимости авторизации
		bool _busy;							// Флаг того, что разбор данных не окончен и новые копировать нельзя
		bool _ready;						// Флаг того, что массив данные приняты и готовы к анализу
		
		uint8_t _data[FdOld::PacketSize];	// Холодный массив данных (Работа в программе)
		
};
