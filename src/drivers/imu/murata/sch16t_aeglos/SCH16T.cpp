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

#include "SCH16T.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <string.h>

namespace
{

uint32_t sch16t_compose_device_id(const I2CSPIDriverConfig &config)
{
	// Same DeviceId composition the posix device::SPI base produces, so the
	// CAL_GYRO*/CAL_ACC* slot matching sees a stable, conventional device_id.
	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SPI;
	device_id.devid_s.bus = config.bus;
	device_id.devid_s.address = (uint8_t)PX4_SPI_DEV_ID(config.spi_devid);
	device_id.devid_s.devtype = DRV_IMU_DEVTYPE_SCH16T;
	return device_id.devid;
}

} // namespace

SCH16T::SCH16T(const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	_spi_bus(config.bus),
	_spi_cs((int)PX4_SPI_DEV_ID(config.spi_devid)),
	// ROTATION_NONE by design: the flight-validated sensor-to-body matrix is applied
	// in-driver (sch16t_px4_apply_sensor_to_body), exactly once.
	_px4_accel(sch16t_compose_device_id(config), ROTATION_NONE),
	_px4_gyro(sch16t_compose_device_id(config), ROTATION_NONE)
{
	// Samples are converted to SI in-driver and published with scale 1.0. The
	// wrappers clamp their clipping threshold to INT16_MAX in *pre-scale units*
	// (PX4Accelerometer.cpp: _clip_limit = constrain(range/scale*0.999, 0,
	// INT16_MAX)); feeding 20-bit raw counts would clamp the accel threshold to
	// 32767 LSB = 10.24 m/s^2 and flag clipping on any modest climb, feeding
	// EKF2's bad-accel logic. With scale 1.0 the threshold is the true device
	// range in SI and "clipping" means device saturation, as intended.
	_px4_gyro.set_scale(1.f);
	_px4_gyro.set_range((float)SCH16T_PX4_GYRO_RANGE_RAD);		// +/-300 deg/s
	_px4_accel.set_scale(1.f);
	_px4_accel.set_range((float)SCH16T_PX4_ACCEL_RANGE_MPS2);	// +/-80 m/s^2
}

SCH16T::~SCH16T()
{
	perf_free(_transfer_error_perf);
	perf_free(_crc_error_perf);
	perf_free(_decode_error_perf);
	perf_free(_duplicate_perf);
	perf_free(_gap_perf);
	perf_free(_missed_slot_perf);
	perf_free(_saturation_perf);
	perf_free(_failure_high_water_perf);
}

int SCH16T::init()
{
	// muORB instance guard: apps-side instance 0 of sensor_accel/sensor_gyro must be
	// the bridged DSP ICM stream. sch16t_main blocks the start until instance 0
	// exists; this assertion catches any ordering violation after our
	// PX4Accelerometer/PX4Gyroscope advertised at construction.
	const int accel_instance = _px4_accel.get_instance();
	const int gyro_instance = _px4_gyro.get_instance();

	if (accel_instance <= 0 || gyro_instance <= 0) {
		PX4_ERR("refusing to run as sensor instance 0 (accel %d, gyro %d): instance 0 belongs to the bridged DSP IMU",
			accel_instance, gyro_instance);
		return PX4_ERROR;
	}

	// Fail-closed gate on the golden configuration manifest BEFORE touching hardware.
	if (!sch16t_config_manifest_valid()) {
		PX4_ERR("config manifest validation failed - not touching hardware");
		return PX4_ERROR;
	}

	char dev_path[16];
	snprintf(dev_path, sizeof(dev_path), "/dev/spidev%i.%i", _spi_bus, _spi_cs);

	if (_transport.open(dev_path, SCH16T_BRINGUP_SPI_HZ) != 0) {
		PX4_ERR("failed to open %s", dev_path);
		return PX4_ERROR;
	}

	if (Bringup() != 0) {
		PX4_ERR("bringup failed after %u attempts", BRINGUP_ATTEMPTS);
		_transport.close();
		return PX4_ERROR;
	}

	PX4_INFO("SCH16T SN %04X-%04X-%04X streaming on %s at %u Hz poll, accel instance %d, gyro instance %d",
		 _identity.serial_id1, _identity.serial_id2, _identity.serial_id3,
		 dev_path, 1000000 / SAMPLE_INTERVAL_US, accel_instance, gyro_instance);

	ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);

	return PX4_OK;
}

int SCH16T::SoftReset()
{
	const uint64_t command = sch16t_soft_reset_command();
	uint64_t response = 0;
	return _transport.transfer_batch(&command, 1, &response);
}

int SCH16T::RunIdentityCycle(uint64_t responses[SCH16T_IDENTITY_FRAME_COUNT], struct sch16t_identity *identity)
{
	uint64_t commands[SCH16T_IDENTITY_FRAME_COUNT];
	sch16t_identity_commands(commands);

	if (_transport.transfer_batch(commands, SCH16T_IDENTITY_FRAME_COUNT, responses) != 0) {
		return -1;
	}

	struct sch16t_proto_validation validation;

	if (!sch16t_validate_identity_cycle(responses, identity, &validation)) {
		PX4_WARN("identity cycle invalid (crc %u value %u): COMP_ID 0x%04X ASIC_ID 0x%04X",
			 validation.crc_errors, validation.value_errors, identity->comp_id, identity->asic_id);
		return -1;
	}

	return 0;
}

int SCH16T::BringupAttempt(uint64_t identity_responses[SCH16T_IDENTITY_FRAME_COUNT])
{
	struct sch16t_proto_validation validation;
	struct sch16t_identity identity;

	if (_transport.set_speed(SCH16T_BRINGUP_SPI_HZ) != 0) {
		return -1;
	}

	// Identity cycle #1 (COMP_ID 0x0023 / ASIC_ID 0x0021 enforced by the validator)
	if (RunIdentityCycle(identity_responses, &identity) != 0) {
		return -1;
	}

	// Soft reset, then the qualified 300 ms recovery wait
	if (SoftReset() != 0) {
		return -1;
	}

	px4_usleep(SCH16T_RESET_WAIT_MS * 1000);

	// Identity cycle #2: the inspected response frames (1..6; frame 0 is the
	// out-of-frame priming reply and answers a different preceding command in
	// each cycle) must be byte-identical to the first cycle.
	uint64_t identity_after_reset[SCH16T_IDENTITY_FRAME_COUNT];

	if (RunIdentityCycle(identity_after_reset, &identity) != 0) {
		return -1;
	}

	if (memcmp(&identity_responses[1], &identity_after_reset[1],
		   (SCH16T_IDENTITY_FRAME_COUNT - 1) * sizeof(uint64_t)) != 0) {
		PX4_WARN("identity not byte-identical across soft reset");
		return -1;
	}

	// The seven golden configuration writes (manifest-gated; never CTRL_USER_IF)
	uint64_t config_writes[SCH16T_CONFIG_WRITE_COUNT];
	uint64_t write_responses[SCH16T_CONFIG_WRITE_COUNT];
	sch16t_config_write_commands(config_writes);

	if (_transport.transfer_batch(config_writes, SCH16T_CONFIG_WRITE_COUNT, write_responses) != 0) {
		return -1;
	}

	px4_usleep(SCH16T_EN_SENSOR_SETTLE_MS * 1000);

	// Status sweep #1, EOI, 5 ms, sweeps #2 and #3; validate the THIRD sweep.
	uint64_t status_commands[SCH16T_STATUS_FRAME_COUNT];
	uint64_t status_responses[SCH16T_STATUS_FRAME_COUNT];
	sch16t_status_read_commands(status_commands);

	if (_transport.transfer_batch(status_commands, SCH16T_STATUS_FRAME_COUNT, status_responses) != 0) {
		return -1;
	}

	const uint64_t eoi = sch16t_eoi_command();
	uint64_t eoi_response = 0;

	if (_transport.transfer_batch(&eoi, 1, &eoi_response) != 0) {
		return -1;
	}

	px4_usleep(SCH16T_EOI_SETTLE_MS * 1000);

	if (_transport.transfer_batch(status_commands, SCH16T_STATUS_FRAME_COUNT, status_responses) != 0) {
		return -1;
	}

	if (_transport.transfer_batch(status_commands, SCH16T_STATUS_FRAME_COUNT, status_responses) != 0) {
		return -1;
	}

	if (!sch16t_validate_status_responses(status_responses, &validation)) {
		PX4_WARN("status validation failed (crc %u value %u)", validation.crc_errors, validation.value_errors);
		return -1;
	}

	// Configuration readback (12-frame double-read)
	uint64_t config_read_commands[SCH16T_CONFIG_READ_FRAME_COUNT];
	uint64_t config_read_responses[SCH16T_CONFIG_READ_FRAME_COUNT];
	sch16t_config_read_commands(config_read_commands);

	if (_transport.transfer_batch(config_read_commands, SCH16T_CONFIG_READ_FRAME_COUNT, config_read_responses) != 0) {
		return -1;
	}

	if (!sch16t_validate_config_responses(config_read_responses, &validation)) {
		PX4_WARN("config readback failed (crc %u value %u)", validation.crc_errors, validation.value_errors);
		return -1;
	}

	// Streaming speed, then one priming capture batch, discarded: it starts the
	// out-of-frame reply pipeline (rolling temperature) at 5 MHz.
	if (_transport.set_speed(SCH16T_DEFAULT_SPI_HZ) != 0) {
		return -1;
	}

	uint64_t capture_commands[SCH16T_CAPTURE_FRAME_COUNT];
	uint64_t capture_responses[SCH16T_CAPTURE_FRAME_COUNT];
	sch16t_capture_read_commands(capture_commands);

	if (_transport.transfer_batch(capture_commands, SCH16T_CAPTURE_FRAME_COUNT, capture_responses) != 0) {
		return -1;
	}

	_identity = identity;
	_primed = true;
	_bracket_valid = false;

	return 0;
}

int SCH16T::Bringup()
{
	uint64_t identity_responses[SCH16T_IDENTITY_FRAME_COUNT];

	for (unsigned attempt = 1; attempt <= BRINGUP_ATTEMPTS; ++attempt) {
		if (BringupAttempt(identity_responses) == 0) {
			return 0;
		}

		PX4_WARN("bringup attempt %u/%u failed", attempt, BRINGUP_ATTEMPTS);

		if (attempt < BRINGUP_ATTEMPTS) {
			// Soft reset between attempts, at bringup speed
			(void)_transport.set_speed(SCH16T_BRINGUP_SPI_HZ);
			(void)SoftReset();
			px4_usleep(SCH16T_RESET_WAIT_MS * 1000);
		}
	}

	return -1;
}

int SCH16T::CaptureBatch(uint64_t responses[SCH16T_CAPTURE_FRAME_COUNT], hrt_abstime &timestamp_mid)
{
	uint64_t commands[SCH16T_CAPTURE_FRAME_COUNT];
	sch16t_capture_read_commands(commands);

	// hrt IS the DSP flight-stack clock on this platform; sample it around the
	// single batch ioctl and timestamp at the midpoint.
	const hrt_abstime before = hrt_absolute_time();
	const int ret = _transport.transfer_batch(commands, SCH16T_CAPTURE_FRAME_COUNT, responses);
	const hrt_abstime after = hrt_absolute_time();

	timestamp_mid = before + (after - before) / 2;
	return ret;
}

void SCH16T::AccountMissedSlots(const hrt_abstime &cycle_start)
{
	if (_last_cycle_start != 0) {
		const hrt_abstime elapsed = cycle_start - _last_cycle_start;
		const uint64_t slots = (elapsed + SAMPLE_INTERVAL_US / 2) / SAMPLE_INTERVAL_US;

		if (slots > 1) {
			_missed_slots += slots - 1;
			perf_set_count(_missed_slot_perf, _missed_slots);
		}
	}

	_last_cycle_start = cycle_start;
}

void SCH16T::CycleFailed(perf_counter_t counter)
{
	perf_count(counter);

	// After an error the N+1 reply pipeline may be out of phase: drop priming so
	// the next cycle issues a discarded re-priming batch before publishing again.
	_primed = false;

	++_consecutive_failures;

	if (_consecutive_failures > _consecutive_failure_high_water) {
		_consecutive_failure_high_water = _consecutive_failures;
		perf_set_count(_failure_high_water_perf, _consecutive_failure_high_water);
	}

	if (_consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
		PX4_ERR("%u consecutive cycle failures - stopping publication and exiting (sensor voting fails over)",
			_consecutive_failures);
		_failed = true;
		exit_and_cleanup();
	}
}

void SCH16T::RunImpl()
{
	if (_failed) {
		return;
	}

	const hrt_abstime cycle_start = hrt_absolute_time();
	AccountMissedSlots(cycle_start);

	uint64_t capture_responses[SCH16T_CAPTURE_FRAME_COUNT];
	hrt_abstime timestamp_sample = 0;

	if (CaptureBatch(capture_responses, timestamp_sample) != 0) {
		CycleFailed(_transfer_error_perf);
		return;
	}

	if (!_primed) {
		// Re-priming batch: replies may be out of phase, discard entirely.
		_primed = true;
		return;
	}

	uint64_t sample_responses[SCH16T_SAMPLE_FRAME_COUNT];
	sch16t_reorder_capture_to_sample(capture_responses, sample_responses);

	struct sch16t_sample sample;
	struct sch16t_proto_validation validation;

	if (!sch16t_decode_sample_responses(sample_responses, &sample, &validation)) {
		if (validation.crc_errors > 0) {
			perf_count(_crc_error_perf);
		}

		CycleFailed(_decode_error_perf);
		return;
	}

	struct sch16t_data_counter_bracket bracket;

	if (!sch16t_decode_bracketed_data_counter(capture_responses, &bracket, &validation)) {
		if (validation.crc_errors > 0) {
			perf_count(_crc_error_perf);
		}

		CycleFailed(_decode_error_perf);
		return;
	}

	// Transport and decode are healthy for this cycle.
	_consecutive_failures = 0;

	if (_bracket_valid) {
		struct sch16t_counter_vector vector;
		const uint64_t elapsed_ns = (timestamp_sample - _previous_timestamp) * 1000;

		switch (sch16t_px4_classify_bracket(&_previous_bracket, &bracket, elapsed_ns, &vector)) {
		case SCH16T_PX4_PACING_SKIP:
			// Same device epoch: the normal ~8% over-poll case. Nothing published;
			// the previous bracket/timestamp stay anchored at the epoch's first
			// observation.
			perf_count(_duplicate_perf);
			return;

		case SCH16T_PX4_PACING_PUBLISH_GAP:
			perf_count(_gap_perf);
			break;

		case SCH16T_PX4_PACING_PUBLISH:
		default:
			break;
		}
	}

	_previous_bracket = bracket;
	_previous_timestamp = timestamp_sample;
	_bracket_valid = true;

	// Saturation: publish anyway; the +/-300 deg/s / +/-80 m/s^2 ranges set on the
	// wrappers make their internal clip detection fire on pinned samples, and the
	// event is accounted here.
	int gyro_saturated = 0;
	int accel_saturated = 0;
	sch16t_px4_saturation_flags(sample_responses, &gyro_saturated, &accel_saturated);

	if (gyro_saturated || accel_saturated) {
		perf_count(_saturation_perf);
	}

	const float temperature_c = (float)(sample.temperature * SCH16T_PX4_TEMP_C_PER_LSB);
	const uint32_t error_count = (uint32_t)(perf_event_count(_transfer_error_perf) +
						perf_event_count(_crc_error_perf) +
						perf_event_count(_decode_error_perf));

	// Sensor-to-body rotation applied exactly once (matrix, ROTATION_NONE in PX4),
	// then LSB -> SI here so the wrappers see scale-1.0 SI values (see ctor note
	// on the INT16_MAX clip-limit clamp).
	float gyro_body[3];
	float accel_body[3];
	sch16t_px4_apply_sensor_to_body(sample.gyro, gyro_body);
	sch16t_px4_apply_sensor_to_body(sample.accel, accel_body);

	for (int i = 0; i < 3; i++) {
		gyro_body[i] = (float)((double)gyro_body[i] * SCH16T_PX4_GYRO_RAD_PER_LSB);
		accel_body[i] = (float)((double)accel_body[i] * SCH16T_PX4_ACCEL_MPS2_PER_LSB);
	}

	_px4_gyro.set_error_count(error_count);
	_px4_gyro.set_temperature(temperature_c);
	_px4_gyro.update(timestamp_sample, gyro_body[0], gyro_body[1], gyro_body[2]);

	_px4_accel.set_error_count(error_count);
	_px4_accel.set_temperature(temperature_c);
	_px4_accel.update(timestamp_sample, accel_body[0], accel_body[1], accel_body[2]);

	++_published_samples;

	if (_publish_first_timestamp == 0) {
		_publish_first_timestamp = timestamp_sample;
	}
}

void SCH16T::print_status()
{
	I2CSPIDriverBase::print_status();

	char serial_str[14];
	snprintf(serial_str, sizeof(serial_str), "%05u%01X%04X",
		 _identity.serial_id2, _identity.serial_id1 & 0x000F, _identity.serial_id3);

	PX4_INFO("serial: %s (SN words %04X %04X %04X), COMP_ID 0x%04X ASIC_ID 0x%04X",
		 serial_str, _identity.serial_id1, _identity.serial_id2, _identity.serial_id3,
		 _identity.comp_id, _identity.asic_id);
	PX4_INFO("uORB instances: accel %d, gyro %d (instance 0 is the bridged DSP IMU)",
		 _px4_accel.get_instance(), _px4_gyro.get_instance());

	if (_failed) {
		PX4_ERR("FAILED: stopped after %u consecutive cycle failures", _consecutive_failure_high_water);
	}

	const hrt_abstime now = hrt_absolute_time();

	if (_publish_first_timestamp != 0 && now > _publish_first_timestamp) {
		const double average_hz = (double)_published_samples * 1e6 / (double)(now - _publish_first_timestamp);
		PX4_INFO("poll %u us (%u Hz), published %llu samples, average %.1f Hz",
			 (unsigned)SAMPLE_INTERVAL_US, 1000000 / SAMPLE_INTERVAL_US,
			 (unsigned long long)_published_samples, average_hz);

		if (_status_timestamp != 0 && now > _status_timestamp) {
			const double recent_hz = (double)(_published_samples - _status_published_samples) * 1e6 /
						 (double)(now - _status_timestamp);
			PX4_INFO("publish rate since last status: %.1f Hz", recent_hz);
		}

		_status_timestamp = now;
		_status_published_samples = _published_samples;
	}

	perf_print_counter(_transfer_error_perf);
	perf_print_counter(_crc_error_perf);
	perf_print_counter(_decode_error_perf);
	perf_print_counter(_duplicate_perf);
	perf_print_counter(_gap_perf);
	perf_print_counter(_missed_slot_perf);
	perf_print_counter(_saturation_perf);
	perf_print_counter(_failure_high_water_perf);
}

