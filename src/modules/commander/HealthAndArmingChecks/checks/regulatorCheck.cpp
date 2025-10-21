/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "regulatorCheck.hpp"

using namespace time_literals;

void RegulatorChecks::checkAndReport(const Context &context, Report &reporter)
{
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime regulator_telemetry_timeout = 2_s;

	regulator_report_s regulator_report;

	if (_regulator_report_sub.copy(&regulator_report) && (now < regulator_report.timestamp + regulator_telemetry_timeout)) {

		checkRegulatorVoltages(context, reporter, regulator_report);

		reporter.setIsPresent(health_component_t::regulators);

	} else if (_param_regulators_checks_required.get()
		   && now - _start_time > 5_s) { // Wait a bit after startup to allow regulator's to init

		/* EVENT
		 * @description
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_CHK_REGS</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure(NavModes::All, health_component_t::regulators, events::ID("check_regulator_telem_missing"),
				       events::Log::Critical, "Regulator telemetry missing");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Regulator telemetry missing");
		}
	}
}

void RegulatorChecks::checkRegulatorVoltages(const Context &context, Report &reporter,
		const regulator_report_s &regulator_report)
{

	for (size_t index = 0; index < (sizeof(regulator_report.voltage) / sizeof(regulator_report.voltage[0])); index++) {
		if (fabsf(regulator_report.voltage[index] - expected_voltages[index]) > 1.0f) { //Check if more than 1V deviation

			/* EVENT
			 * @description
			 * <profile name="dev">
			 * This check can be configured via <param>COM_ARM_CHK_REGS</param> parameter.
			 * </profile>
			 */
			reporter.healthFailure<uint8_t, float, float>(NavModes::All, health_component_t::regulators,
					events::ID("check_regulator_voltages"),
					events::Log::Critical, "Regulator {1} voltage fault: {2:.1}V (expected {3:.1}V)",
					(uint8_t)index, regulator_report.voltage[index], expected_voltages[index]);

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Regulator %d voltage fault", (int)index);
			}

		}
	}

}
