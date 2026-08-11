/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file SCH16T.hpp
 *
 * Driver for the Murata SCH16T-K01 IMU on Linux spidev (VOXL2 apps processor).
 *
 * Integration skeleton (module registration, I2CSPIDriver work-queue scheduling,
 * PX4Accelerometer/PX4Gyroscope publication) follows the upstream v1.16 sch16t
 * driver; the SPI transaction layer, register configuration, and bringup sequence
 * are the qualified Aeglos SafeSPI protocol core (sch16t_protocol.{c,h}, linked
 * verbatim) and deliberately differ from upstream. CTRL_USER_IF (0x33) is never
 * written: that write reproducibly disrupted SafeSPI responses on this unit.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include "Sch16tSpidev.hpp"
#include "sch16t_protocol.h"
#include "sch16t_px4_logic.h"

class SCH16T : public I2CSPIDriver<SCH16T>
{
public:
	SCH16T(const I2CSPIDriverConfig &config);
	~SCH16T() override;

	static void print_usage();

	int init();

	void RunImpl();
	void print_status() override;

private:
	/* Fixed host poll interval: ~806 Hz, deliberately over the F_PRIM device-ODR
	 * envelope (690.6..784.4 Hz); the bracketed data counter discards duplicate
	 * epochs so publication lands at the true device rate (~743 Hz). */
	static constexpr uint32_t SAMPLE_INTERVAL_US{1240};
	static constexpr unsigned BRINGUP_ATTEMPTS{3};
	static constexpr unsigned MAX_CONSECUTIVE_FAILURES{64};

	int Bringup();
	int BringupAttempt(uint64_t identity_responses[SCH16T_IDENTITY_FRAME_COUNT]);
	int RunIdentityCycle(uint64_t responses[SCH16T_IDENTITY_FRAME_COUNT], struct sch16t_identity *identity);
	int SoftReset();
	int CaptureBatch(uint64_t responses[SCH16T_CAPTURE_FRAME_COUNT], hrt_abstime &timestamp_mid);
	void AccountMissedSlots(const hrt_abstime &cycle_start);
	void CycleFailed(perf_counter_t counter);

	Sch16tSpidev _transport{};

	const int _spi_bus;
	const int _spi_cs;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	struct sch16t_identity _identity {};

	struct sch16t_data_counter_bracket _previous_bracket {};
	hrt_abstime _previous_timestamp{0};
	bool _bracket_valid{false};

	/* Out-of-frame reply pipeline primed (a batch was issued and its replies
	 * discarded). Dropped on any transfer/decode failure: after an error the
	 * N+1 reply pipeline may be out of phase. */
	bool _primed{false};

	unsigned _consecutive_failures{0};
	unsigned _consecutive_failure_high_water{0};
	bool _failed{false};

	hrt_abstime _last_cycle_start{0};
	uint64_t _missed_slots{0};

	hrt_abstime _publish_first_timestamp{0};
	uint64_t _published_samples{0};
	hrt_abstime _status_timestamp{0};
	uint64_t _status_published_samples{0};

	perf_counter_t _transfer_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": transfer errors")};
	perf_counter_t _crc_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": crc errors")};
	perf_counter_t _decode_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": decode errors")};
	perf_counter_t _duplicate_perf{perf_alloc(PC_COUNT, MODULE_NAME": duplicate epochs")};
	perf_counter_t _gap_perf{perf_alloc(PC_COUNT, MODULE_NAME": epoch gaps")};
	perf_counter_t _missed_slot_perf{perf_alloc(PC_COUNT, MODULE_NAME": missed slots")};
	perf_counter_t _saturation_perf{perf_alloc(PC_COUNT, MODULE_NAME": saturation events")};
	perf_counter_t _failure_high_water_perf{perf_alloc(PC_COUNT, MODULE_NAME": consecutive-failure high-water")};
	perf_counter_t _run_elapsed_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": run elapsed")};
	perf_counter_t _capture_elapsed_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": capture ioctl elapsed")};
	perf_counter_t _publish_elapsed_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": uORB publish elapsed")};
};
