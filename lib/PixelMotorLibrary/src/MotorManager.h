#pragma once
#include <inttypes.h>
#include <string.h>
#include <MotorDeviceInterface.h>
#include <MotorData.h>

class MotorManager
{
	static constexpr uint8_t _max_dev = 2;
	
	using callback_tx_t = void (*)(uint8_t idx, const uint8_t *raw, const uint16_t length);
	using callback_error_t = void (*)(uint8_t idx, MotorDeviceInterface::error_code_t code);

	public:

		MotorManager(callback_tx_t tx, callback_error_t error) : _callback_tx(tx), _callback_error(error)
		{
			memset(_device, 0x00, sizeof(_device));
			memset(common_data, 0x00, sizeof(common_data));
			memset(common_data_ready, 0x00, sizeof(common_data_ready));
			
			return;
		}
		
		void SetModel(uint8_t idx, MotorDeviceInterface &device);
		void Tick(uint32_t &time);
		void RawRx(uint8_t idx, const uint8_t *raw, const uint8_t length);
		void RawTx(uint8_t idx, const uint8_t *raw, const uint8_t length);
		void ResetCommonData(uint8_t idx);
		
		MotorManagerData::common_data_t common_data[_max_dev];
		bool common_data_ready[_max_dev];
		
	private:

		callback_tx_t _callback_tx;
		callback_error_t _callback_error;
		
		MotorDeviceInterface *_device[_max_dev];

};
