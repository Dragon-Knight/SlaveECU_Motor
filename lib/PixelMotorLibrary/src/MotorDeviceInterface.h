#pragma once
#include <inttypes.h>

class MotorManager;

class MotorDeviceInterface
{
	public:
		
		MotorDeviceInterface() : _initiated(false), _manager(nullptr), _idx(0)
		{

		}
		
		virtual void Init() = 0;
		virtual void Tick(uint32_t &time) = 0;
		virtual int8_t RawRx(const uint8_t *raw, const uint8_t length) = 0;
		
		bool IsInitiated()
		{
			return _initiated;
		}
		
		void PrepareInit(MotorManager *obj, uint8_t idx)
		{
			_initiated = true;
			_manager = obj;
			_idx = idx;
		}
	
	protected:

		void _ReverseArray(uint8_t *array, uint8_t length)
		{
			uint8_t i = 0;
			uint8_t j = length - 1;
			uint8_t temp;
			while(i < j)
			{
				temp = array[i];
				array[i] = array[j];
				array[j] = temp;
				
				i++;
				j--;
			}

			return;
		}
		
		bool _initiated;			// Флаг того, что обхект был добавлен в менеджер
		MotorManager *_manager;		// Указатель на менеджер
		uint8_t _idx;				// Индекс порт
	
	private:
		
};
