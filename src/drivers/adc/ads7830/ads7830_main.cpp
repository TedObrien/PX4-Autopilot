/****************************************************************************
 *
 *   Copyright (C) 2020-2021 PX4 Development Team. All rights reserved.
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

/**
 * @file ads7830_main.cpp
 * @author SalimTerryLi
 *
 * Driver for the ADS7830 connected via I2C.
 */

#include "ADS7830.h"
#include <px4_platform_common/module.h>
#include <drivers/drv_adc.h>

ADS7830::ADS7830(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))

{
	_adc7830_report.device_id = this->get_device_id();
	_adc7830_report.resolution = 256; // 8-bit ADC
	_adc7830_report.v_ref = 5.0f; // Internal reference is 2.5V we are using 5V external ref via REF pin

	for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
		_adc7830_report.channel_id[i] = -1;
	}
}

ADS7830::~ADS7830()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);

}


void ADS7830::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();	// nothing to do
}

void ADS7830::RunImpl()
{
	if (should_exit()) {
		return;
	}

	perf_begin(_cycle_perf);
	_adc7830_report.timestamp = hrt_absolute_time();

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
			_adc7830_report.channel_id[channel_index] = static_cast<int>(channel_index);
			_adc7830_report.raw_data[channel_index] = value;
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
			_to_adc7830_report.publish(_adc7830_report);
		}
	}

	perf_end(_cycle_perf);
}

void ADS7830::print_usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver to enable an external ADS7830 8 channel ADC connected via I2C. Used by Airial UAS to monitor voltages of redundant regulators.

It is enabled/disabled using the
[ADC_ADS7830_EN]

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ads7830", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x4a);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void ADS7830::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_errors);

}

extern "C" int ads7830_main(int argc, char *argv[])
{
	using ThisDriver = ADS7830;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = 0x4a; // Default address 0x48 but ads1115 is already on this. A0 pulled low, A1 pulled high gives this adress

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ADC_DEVTYPE_ADS7830);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
