/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ADS7830.h"
#include <px4_platform_common/module.h>
#include <drivers/drv_adc.h>
#include <cassert>

ADS7830::ADS7830(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))

{
	_regulator_report.device_id = get_device_id();
	_regulator_report.resolution = 256; // 8-bit ADC
	_regulator_report.v_ref = 5.0f; // Internal reference is 2.5V we are using 5V external ref via REF pin

	for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
		_regulator_report.channel_id[i] = -1;
	}
}

ADS7830::~ADS7830()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);

}

void ADS7830::RunImpl()
{
	if (should_exit()) {
		return;
	}

	perf_begin(_cycle_perf);
	_regulator_report.timestamp = hrt_absolute_time();

	int16_t value = 0;
	ChannelSelection ch = cycleMeasure(&value);

	if (ch != Invalid) {
		_last_successful_measurement = hrt_absolute_time(); // update timedtamp of last measurement
		unsigned channel_index = static_cast<unsigned>(ch);

		if (!_already_connected) {
			_already_connected = true;
			PX4_INFO("Device ready");
		}

		if (channel_index < PX4_MAX_ADC_CHANNELS) {
			_regulator_report.channel_id[channel_index] = static_cast<int>(channel_index);
			_regulator_report.raw_data[channel_index] = value; // Publish raw value
			_analogue_value = value;
			//Convert 0-255 to between  0-Vref, then mulitply by scale factor to get regulator voltage
			_regulator_report.voltage[channel_index] = (static_cast<float>(_analogue_value)) * 0.06f;
		}

	} else {
		// check for timeout
		if (hrt_elapsed_time(&_last_successful_measurement) > MEASUREMENT_TIMEOUT_US) {
			// Print message once on disconnect
			if (_already_connected) {
				PX4_ERR("No valid measurements for %.1f seconds - device may be disconnected", MEASUREMENT_TIMEOUT_US / 1e6);
				_already_connected = false;
			}

			perf_count(_comms_errors);
		}
	}

	// Only publish if we have recent data
	if (hrt_elapsed_time(&_last_successful_measurement) <= MEASUREMENT_TIMEOUT_US) {

		// Publish after all channels have been sampled
		if (++_channel_cycle_count >= 8) {
			_channel_cycle_count = 0;
			_regulator_report_pub.publish(_regulator_report);
		}
	}

	perf_end(_cycle_perf);
}

int ADS7830::init()
{

	int ret = I2C::init();

	if (ret != PX4_OK) {

		PX4_ERR("failed to init i2c");

		return ret;
	}

	// Simple command - single-ended, channel 0, reference OFF, ADC ON
	// If you want to use internal 2.5V ref, replace ADS7830_CMD_PD_REF_OFF_ADC_ON with ADS7830_CMD_PD_REF_ON_ADC_ON
	uint8_t config = ADS7830_CMD_SD_SINGLE_ENDED | ADS7830_CMD_PD_REF_OFF_ADC_ON | ADS7830_CMD_CH0;
	ret = transfer(&config, 1, nullptr, 0);

	if (ret != PX4_OK) {
		PX4_ERR("writeReg failed (%i)", ret);
		return ret;
	}

	_regulator_report_pub.advertise();

	px4_usleep(10000); // Sleep for 10ms after turning REF ON
	PX4_DEBUG("succesfully init i2c");

	ScheduleOnInterval(SAMPLE_INTERVAL / 4, SAMPLE_INTERVAL / 4);

	return PX4_OK;
}


int ADS7830::probe()
{

	// Simple command - single-ended, channel 0, reference OFF, ADC ON
	uint8_t cmd = ADS7830_CMD_SD_SINGLE_ENDED | ADS7830_CMD_PD_REF_OFF_ADC_ON | ADS7830_CMD_CH0;
	uint8_t data[1];

	// Attempt to read conversion from channel 0
	int ret = transfer(&cmd, 1, data, 1);

	if (ret != PX4_OK) {
		PX4_ERR("I2C transfer failed (%i)", ret);
		return ret;
	}

	PX4_INFO("Device probe successful, Channel 0 value: %d", data[0]);
	return PX4_OK;
}


int ADS7830::setChannel(ChannelSelection ch)
{
	uint8_t cmd = ADS7830_CMD_SD_SINGLE_ENDED | ADS7830_CMD_PD_REF_ON_ADC_ON;

	switch (ch) {
	case A0: cmd |= ADS7830_CMD_CH0; break;

	case A1: cmd |= ADS7830_CMD_CH1; break;

	case A2: cmd |= ADS7830_CMD_CH2; break;

	case A3: cmd |= ADS7830_CMD_CH3; break;

	case A4: cmd |= ADS7830_CMD_CH4; break;

	case A5: cmd |= ADS7830_CMD_CH5; break;

	case A6: cmd |= ADS7830_CMD_CH6; break;

	case A7: cmd |= ADS7830_CMD_CH7; break;

	default: return -EINVAL;
	}

	// Write command byte to start conversion on selected channel
	return transfer(&cmd, 1, nullptr, 0);
}

int ADS7830::readConversionResult(uint8_t *value)
{
	// Read the conversion result from the previous command
	return transfer(nullptr, 0, value, 1);
}


ADS7830::ChannelSelection ADS7830::cycleMeasure(int16_t *value)
{
	static ChannelSelection current_channel = A0;

	// Set the next channel and start conversion
	int ret = setChannel(current_channel);

	if (ret != PX4_OK) {
		// PX4_ERR("setChannel failed (%i)", ret);
		return Invalid;
	}

	// Small delay to allow conversion to complete
	usleep(20); // Conversion time is typically 5µs

	// Read the conversion result
	uint8_t raw_value = 0;
	ret = readConversionResult(&raw_value);

	if (ret != PX4_OK) {
		PX4_ERR("readConversionResult failed (%i)", ret);
		return Invalid;
	}

	*value = (int16_t)raw_value;
	ChannelSelection measured_channel = current_channel;

	// Cycle to the next channel
	switch (current_channel) {
	case A0: current_channel = A1; break;

	case A1: current_channel = A2; break;

	case A2: current_channel = A3; break;

	case A3: current_channel = A4; break;

	case A4: current_channel = A5; break;

	case A5: current_channel = A6; break;

	case A6: current_channel = A7; break;

	case A7: current_channel = A0; break;

	default: current_channel = A0; break;
	}

	return measured_channel;
}
