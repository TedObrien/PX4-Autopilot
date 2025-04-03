#pragma once

#include <stdint.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/regulator_report.h>
#include <uORB/PublicationMulti.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

// Command Byte Definitions
#define ADS7830_CMD_SD_SINGLE_ENDED    0x80  // Single-ended mode
#define ADS7830_CMD_SD_DIFFERENTIAL    0x00  // Differential mode

// Power Down and Reference Control
#define ADS7830_CMD_PD_MASK           0x0C
#define ADS7830_CMD_PD_REF_OFF_ADC_OFF 0x00  // Power down between conversions
#define ADS7830_CMD_PD_REF_OFF_ADC_ON  0x04  // Internal ref OFF, ADC ON
#define ADS7830_CMD_PD_REF_ON_ADC_OFF  0x08  // Internal ref ON, ADC OFF
#define ADS7830_CMD_PD_REF_ON_ADC_ON   0x0C  // Internal ref ON, ADC ON

// Channel Selection Bits (C2, C1, C0)
#define ADS7830_CMD_CH_MASK           0x70
#define ADS7830_CMD_CH0               0x00  // Channel 0
#define ADS7830_CMD_CH1               0x10  // Channel 1
#define ADS7830_CMD_CH2               0x20  // Channel 2
#define ADS7830_CMD_CH3               0x30  // Channel 3
#define ADS7830_CMD_CH4               0x40  // Channel 4
#define ADS7830_CMD_CH5               0x50  // Channel 5
#define ADS7830_CMD_CH6               0x60  // Channel 6
#define ADS7830_CMD_CH7               0x70  // Channel 7

using namespace time_literals;

class ADS7830 : public device::I2C, public I2CSPIDriver<ADS7830>
{
public:
	ADS7830(const I2CSPIDriverConfig &config);
	~ADS7830() override;

	int init() override;
	static void print_usage();
	void RunImpl();
	int probe() override;

protected:
	void print_status();

private:
	uORB::PublicationMulti<regulator_report_s> _regulator_report_pub{ORB_ID(regulator_report)};

	static const hrt_abstime SAMPLE_INTERVAL{200_ms};
	regulator_report_s _regulator_report{};
	perf_counter_t _cycle_perf;
	perf_counter_t _comms_errors;
	int _channel_cycle_count{0};
	enum ChannelSelection {
		Invalid = -1,
		A0 = 0, A1, A2, A3, A4, A5, A6, A7
	};
	hrt_abstime _last_successful_measurement{0};
	static constexpr hrt_abstime MEASUREMENT_TIMEOUT_US = 2000_ms;
	bool  _already_connected{false};
	int setChannel(ChannelSelection ch);
	int readConversionResult(uint8_t *value);
	ChannelSelection cycleMeasure(int16_t *value);
	int _analogue_value{0};


};
