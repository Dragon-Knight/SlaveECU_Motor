#pragma once

#include <inttypes.h>
#include <string.h>

template <typename T1, typename T2, uint8_t _size> 
class MovingAverage
{
	public:
		
		MovingAverage()
		{
			memset(&_data, 0x00, sizeof(_data));

			return;
		}
		
		void Set(T1 value)
		{
			_data.sum = 0;
			for(uint8_t i = 0; i < _size; ++i)
			{
				_data.buffer[i] = value;
				_data.sum += value;
			}
			
			return;
		}
		
		void Push(T1 value)
		{
			_data.sum -= _data.buffer[_data.idx];
			_data.buffer[_data.idx] = value;
			_data.sum += value;
			_data.idx = (_data.idx + 1) % _size;
			
			return;
		}
		
		T1 Get()
		{
			return _data.sum / _size;
		}
		
	
	private:
		
		struct data_t
		{
			T1 buffer[_size];
			T2 sum;
			uint8_t idx;
		} _data;
		
};
