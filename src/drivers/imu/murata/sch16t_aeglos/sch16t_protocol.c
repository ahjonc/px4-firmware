// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-2-Clause
/*
 * Murata SCH16T SafeSPI protocol helpers (host + kernel).
 *
 * One implementation, linked byte-identically by every consumer of the device, so
 * that no two transports can disagree about CRC, framing, or decode. Conformance
 * is pinned against live-device golden frames by tests/test_protocol_conformance.py.
 */

#include "sch16t_protocol.h"

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/string.h>
#else
#include <limits.h>
#include <stdint.h>
#include <string.h>
#endif

#ifndef UINT64_C
#define UINT64_C(c) c##ULL
#endif
#ifndef UINT32_MAX
#define UINT32_MAX 0xFFFFFFFFU
#endif

struct sch16t_reg_value {
	uint8_t address;
	uint16_t value;
};

static const struct sch16t_reg_value sch16t_configuration[SCH16T_CONFIG_WRITE_COUNT] = {
	{ SCH16T_REG_CTRL_FILT_RATE, SCH16T_CFG_FILT_RATE },
	{ SCH16T_REG_CTRL_FILT_ACC12, SCH16T_CFG_FILT_ACC12 },
	{ SCH16T_REG_CTRL_FILT_ACC3, SCH16T_CFG_FILT_ACC3 },
	{ SCH16T_REG_CTRL_RATE, SCH16T_CFG_CTRL_RATE },
	{ SCH16T_REG_CTRL_ACC12, SCH16T_CFG_CTRL_ACC12 },
	{ SCH16T_REG_CTRL_ACC3, SCH16T_CFG_CTRL_ACC3 },
	{ SCH16T_REG_CTRL_MODE, SCH16T_CTRL_MODE_EN_SENSOR },
};

static const uint8_t sch16t_identity_addrs[SCH16T_IDENTITY_FRAME_COUNT] = {
	SCH16T_REG_COMP_ID,
	SCH16T_REG_ASIC_ID,
	SCH16T_REG_ASIC_ID,
	SCH16T_REG_SN_ID1,
	SCH16T_REG_SN_ID2,
	SCH16T_REG_SN_ID3,
	SCH16T_REG_SN_ID3,
};

static const uint8_t sch16t_status_addrs[SCH16T_STATUS_FRAME_COUNT] = {
	SCH16T_REG_STAT_SUM,	 SCH16T_REG_STAT_SUM_SAT, SCH16T_REG_STAT_COM,
	SCH16T_REG_STAT_RATE_COM, SCH16T_REG_STAT_RATE_X,  SCH16T_REG_STAT_RATE_Y,
	SCH16T_REG_STAT_RATE_Z,	 SCH16T_REG_STAT_ACC_X,	  SCH16T_REG_STAT_ACC_Y,
	SCH16T_REG_STAT_ACC_Z,	 SCH16T_REG_STAT_ACC_Z,
};

static const uint8_t sch16t_sample_addrs[SCH16T_SAMPLE_FRAME_COUNT] = {
	SCH16T_REG_RATE_X2, SCH16T_REG_RATE_Y2, SCH16T_REG_RATE_Z2, SCH16T_REG_ACC_X2,
	SCH16T_REG_ACC_Y2,  SCH16T_REG_ACC_Z2,	SCH16T_REG_TEMP,     SCH16T_REG_TEMP,
};

static const uint8_t sch16t_capture_addrs[SCH16T_CAPTURE_FRAME_COUNT] = {
	SCH16T_REG_RATE_STATUS1, SCH16T_REG_ACC_STATUS1, SCH16T_REG_RATE_X2, SCH16T_REG_RATE_Y2,
	SCH16T_REG_RATE_Z2,	 SCH16T_REG_ACC_X2,	SCH16T_REG_ACC_Y2,  SCH16T_REG_ACC_Z2,
	SCH16T_REG_RATE_STATUS1, SCH16T_REG_ACC_STATUS1, SCH16T_REG_TEMP,
};

uint8_t sch16t_crc8(uint64_t frame)
{
	const uint64_t data = frame & UINT64_C(0xFFFFFFFFFF00);
	uint8_t crc = 0xFF;
	int bit;

	for (bit = 47; bit >= 0; --bit) {
		const uint8_t data_bit = (uint8_t)((data >> bit) & 1U);

		if (crc & 0x80U)
			crc = (uint8_t)(((crc << 1) ^ 0x2FU) ^ data_bit);
		else
			crc = (uint8_t)((crc << 1) | data_bit);
	}
	return crc;
}

uint64_t sch16t_make_read_frame(uint8_t address)
{
	uint64_t frame = ((uint64_t)address << SCH16T_FRAME_ADDRESS_SHIFT) |
			 (UINT64_C(1) << SCH16T_FRAME_FT_BIT);

	return frame | sch16t_crc8(frame);
}

uint64_t sch16t_make_write_frame(uint8_t address, uint16_t value)
{
	uint64_t frame = (UINT64_C(1) << SCH16T_FRAME_RW_BIT) |
			 ((uint64_t)address << SCH16T_FRAME_ADDRESS_SHIFT) |
			 (UINT64_C(1) << SCH16T_FRAME_FT_BIT) |
			 ((uint64_t)value << SCH16T_FRAME_DATA_SHIFT);

	return frame | sch16t_crc8(frame);
}

int sch16t_frame_crc_valid(uint64_t frame)
{
	return (uint8_t)frame == sch16t_crc8(frame);
}

uint16_t sch16t_frame_data16(uint64_t frame)
{
	return (uint16_t)((frame >> SCH16T_FRAME_DATA_SHIFT) & UINT64_C(0xFFFF));
}

int32_t sch16t_frame_data20(uint64_t frame)
{
	uint32_t value = (uint32_t)((frame >> SCH16T_FRAME_DATA_SHIFT) & UINT64_C(0xFFFFF));

	if (value & 0x80000U)
		value |= 0xFFF00000U;
	return (int32_t)value;
}

void sch16t_frame_to_wire_bytes(uint64_t frame, uint8_t output[SCH16T_FRAME_BYTE_COUNT])
{
	unsigned int index;

	for (index = 0; index < SCH16T_FRAME_BYTE_COUNT; ++index)
		output[index] = (uint8_t)(frame >> ((SCH16T_FRAME_BYTE_COUNT - index - 1U) * 8U));
}

uint64_t sch16t_frame_from_wire_bytes(const uint8_t input[SCH16T_FRAME_BYTE_COUNT])
{
	uint64_t frame = 0;
	unsigned int index;

	for (index = 0; index < SCH16T_FRAME_BYTE_COUNT; ++index)
		frame = (frame << 8) | input[index];
	return frame;
}

void sch16t_identity_commands(uint64_t output[SCH16T_IDENTITY_FRAME_COUNT])
{
	unsigned int index;

	for (index = 0; index < SCH16T_IDENTITY_FRAME_COUNT; ++index)
		output[index] = sch16t_make_read_frame(sch16t_identity_addrs[index]);
}

uint64_t sch16t_soft_reset_command(void)
{
	return sch16t_make_write_frame(SCH16T_REG_CTRL_RESET, SCH16T_SPI_SOFT_RESET_VALUE);
}

uint64_t sch16t_eoi_command(void)
{
	return sch16t_make_write_frame(SCH16T_REG_CTRL_MODE, SCH16T_CTRL_MODE_EOI);
}

void sch16t_config_write_commands(uint64_t output[SCH16T_CONFIG_WRITE_COUNT])
{
	unsigned int index;

	for (index = 0; index < SCH16T_CONFIG_WRITE_COUNT; ++index)
		output[index] = sch16t_make_write_frame(sch16t_configuration[index].address,
							sch16t_configuration[index].value);
}

void sch16t_status_read_commands(uint64_t output[SCH16T_STATUS_FRAME_COUNT])
{
	unsigned int index;

	for (index = 0; index < SCH16T_STATUS_FRAME_COUNT; ++index)
		output[index] = sch16t_make_read_frame(sch16t_status_addrs[index]);
}

void sch16t_config_read_commands(uint64_t output[SCH16T_CONFIG_READ_FRAME_COUNT])
{
	unsigned int index;

	for (index = 0; index < SCH16T_MEASUREMENT_CONFIG_COUNT; ++index) {
		output[index * 2U] = sch16t_make_read_frame(sch16t_configuration[index].address);
		output[index * 2U + 1U] = sch16t_make_read_frame(sch16t_configuration[index].address);
	}
}

void sch16t_sample_read_commands(uint64_t output[SCH16T_SAMPLE_FRAME_COUNT])
{
	unsigned int index;

	for (index = 0; index < SCH16T_SAMPLE_FRAME_COUNT; ++index)
		output[index] = sch16t_make_read_frame(sch16t_sample_addrs[index]);
}

void sch16t_capture_read_commands(uint64_t output[SCH16T_CAPTURE_FRAME_COUNT])
{
	unsigned int index;

	for (index = 0; index < SCH16T_CAPTURE_FRAME_COUNT; ++index)
		output[index] = sch16t_make_read_frame(sch16t_capture_addrs[index]);
}

int sch16t_validate_identity_cycle(const uint64_t responses[SCH16T_IDENTITY_FRAME_COUNT],
				   struct sch16t_identity *identity,
				   struct sch16t_proto_validation *validation)
{
	static const unsigned int checked_frames[] = { 1, 2, 3, 4, 5, 6 };
	unsigned int index;

	if (!responses || !identity || !validation)
		return 0;

	memset(validation, 0, sizeof(*validation));
	identity->comp_id = sch16t_frame_data16(responses[1]);
	identity->asic_id = sch16t_frame_data16(responses[2]);
	identity->serial_id1 = sch16t_frame_data16(responses[4]);
	identity->serial_id2 = sch16t_frame_data16(responses[5]);
	identity->serial_id3 = sch16t_frame_data16(responses[6]);

	for (index = 0; index < sizeof(checked_frames) / sizeof(checked_frames[0]); ++index) {
		if (!sch16t_frame_crc_valid(responses[checked_frames[index]]))
			++validation->crc_errors;
	}

	if (identity->comp_id != SCH16T_EXPECTED_COMP_ID)
		++validation->value_errors;
	if (identity->asic_id != SCH16T_EXPECTED_ASIC_ID)
		++validation->value_errors;
	if (sch16t_frame_data16(responses[3]) != SCH16T_EXPECTED_ASIC_ID)
		++validation->value_errors;

	return validation->crc_errors == 0 && validation->value_errors == 0;
}

int sch16t_validate_status_responses(const uint64_t responses[SCH16T_STATUS_FRAME_COUNT],
				     struct sch16t_proto_validation *validation)
{
	unsigned int index;

	if (!responses || !validation)
		return 0;

	memset(validation, 0, sizeof(*validation));
	for (index = 1; index < SCH16T_STATUS_FRAME_COUNT; ++index) {
		if (!sch16t_frame_crc_valid(responses[index]))
			++validation->crc_errors;
		if (sch16t_frame_data16(responses[index]) != SCH16T_STATUS_NORMAL_VALUE)
			++validation->value_errors;
	}
	return validation->crc_errors == 0 && validation->value_errors == 0;
}

int sch16t_validate_config_responses(const uint64_t responses[SCH16T_CONFIG_READ_FRAME_COUNT],
				     struct sch16t_proto_validation *validation)
{
	unsigned int register_index;

	if (!responses || !validation)
		return 0;

	memset(validation, 0, sizeof(*validation));
	for (register_index = 0; register_index < SCH16T_MEASUREMENT_CONFIG_COUNT; ++register_index) {
		const unsigned int response_index = register_index * 2U + 1U;

		if (!sch16t_frame_crc_valid(responses[response_index]))
			++validation->crc_errors;
		if (sch16t_frame_data16(responses[response_index]) !=
		    sch16t_configuration[register_index].value)
			++validation->value_errors;
	}
	return validation->crc_errors == 0 && validation->value_errors == 0;
}

int sch16t_decode_sample_responses(const uint64_t responses[SCH16T_SAMPLE_FRAME_COUNT],
				   struct sch16t_sample *sample,
				   struct sch16t_proto_validation *validation)
{
	int32_t values[7];
	unsigned int index;

	if (!responses || !sample || !validation)
		return 0;

	memset(validation, 0, sizeof(*validation));
	memset(values, 0, sizeof(values));
	for (index = 1; index < SCH16T_SAMPLE_FRAME_COUNT; ++index) {
		const uint64_t response = responses[index];

		if (!sch16t_frame_crc_valid(response))
			++validation->crc_errors;
		if ((response & SCH16T_FRAME_GENERAL_ERROR) != 0 ||
		    (response & SCH16T_FRAME_COMMAND_ERROR) != 0 ||
		    (response & SCH16T_FRAME_DOING_INITIALIZATION) ==
			    SCH16T_FRAME_DOING_INITIALIZATION)
			++validation->status_errors;
		values[index - 1U] = sch16t_frame_data20(response);
	}

	sample->gyro[0] = values[0];
	sample->gyro[1] = values[1];
	sample->gyro[2] = values[2];
	sample->accel[0] = values[3];
	sample->accel[1] = values[4];
	sample->accel[2] = values[5];
	sample->temperature = values[6] >> 4;
	return validation->crc_errors == 0 && validation->status_errors == 0;
}

static int decode_counter_register(uint64_t response, uint16_t *counter,
				   struct sch16t_proto_validation *validation)
{
	uint16_t value;

	if (!sch16t_frame_crc_valid(response)) {
		++validation->crc_errors;
		return 0;
	}
	value = sch16t_frame_data16(response);
	if ((value & SCH16T_DATA_COUNTER_RESERVED_MASK) != 0U) {
		++validation->value_errors;
		return 0;
	}
	*counter = value;
	return 1;
}

int sch16t_decode_bracketed_data_counter(const uint64_t responses[SCH16T_CAPTURE_FRAME_COUNT],
					 struct sch16t_data_counter_bracket *counters,
					 struct sch16t_proto_validation *validation)
{
	uint16_t values[4] = { 0 };
	static const unsigned int indices[4] = { 1U, 2U, 9U, 10U };
	unsigned int index;

	if (!responses || !counters || !validation)
		return 0;

	memset(validation, 0, sizeof(*validation));
	for (index = 0; index < 4U; ++index)
		(void)decode_counter_register(responses[indices[index]], &values[index], validation);

	if (validation->crc_errors || validation->value_errors)
		return 0;

	counters->rate_start = values[0];
	counters->accel_start = values[1];
	counters->rate_end = values[2];
	counters->accel_end = values[3];
	return 1;
}

void sch16t_reorder_capture_to_sample(const uint64_t capture[SCH16T_CAPTURE_FRAME_COUNT],
				      uint64_t sample[SCH16T_SAMPLE_FRAME_COUNT])
{
	unsigned int index;

	sample[0] = 0;
	for (index = 0; index < 6U; ++index)
		sample[index + 1U] = capture[index + 3U];
	sample[7] = capture[0];
}

int sch16t_counter_epoch_window(uint64_t elapsed_ns, uint64_t *low, uint64_t *high)
{
	uint64_t floor_epochs;
	uint64_t ceil_epochs;

	if (!low || !high)
		return 0;
	*low = 0;
	*high = 0;
	if (elapsed_ns == 0 || elapsed_ns > SCH16T_COUNTER_MAX_INTERVAL_NS)
		return 0;

	floor_epochs = (elapsed_ns * SCH16T_DEVICE_ODR_MIN_MILLIHZ) / SCH16T_COUNTER_EPOCH_SCALE;
	ceil_epochs = (elapsed_ns * SCH16T_DEVICE_ODR_MAX_MILLIHZ + SCH16T_COUNTER_EPOCH_SCALE - 1U) /
		      SCH16T_COUNTER_EPOCH_SCALE;
	*low = floor_epochs > (uint64_t)SCH16T_COUNTER_CHANNEL_SKEW
		       ? floor_epochs - SCH16T_COUNTER_CHANNEL_SKEW
		       : 0U;
	*high = ceil_epochs + SCH16T_COUNTER_CHANNEL_SKEW;
	return 1;
}

static unsigned int resolve_channel_advance(uint8_t residue, uint64_t low, uint64_t high,
					    uint32_t *delta)
{
	unsigned int branches = 0;
	uint64_t candidate;

	*delta = residue;
	for (candidate = residue; candidate <= high; candidate += (uint64_t)SCH16T_COUNTER_WRAP_SPAN) {
		if (candidate < low)
			continue;
		if (branches == 0)
			*delta = (uint32_t)candidate;
		++branches;
	}
	return branches;
}

enum sch16t_counter_step sch16t_counter_vector_step(uint16_t previous_rate, uint16_t previous_accel,
						    uint16_t current_rate, uint16_t current_accel,
						    uint64_t elapsed_ns,
						    struct sch16t_counter_vector *vector)
{
	unsigned int axis;
	uint64_t low = 0;
	uint64_t high = 0;
	int windowed;
	uint32_t smallest = UINT32_MAX;
	uint32_t largest = 0;
	int resolved;
	unsigned int channel;

	if (!vector)
		return SCH16T_COUNTER_GAP;

	memset(vector, 0, sizeof(*vector));
	if ((previous_rate & SCH16T_DATA_COUNTER_RESERVED_MASK) != 0U ||
	    (previous_accel & SCH16T_DATA_COUNTER_RESERVED_MASK) != 0U ||
	    (current_rate & SCH16T_DATA_COUNTER_RESERVED_MASK) != 0U ||
	    (current_accel & SCH16T_DATA_COUNTER_RESERVED_MASK) != 0U)
		return SCH16T_COUNTER_GAP;

	for (axis = 0; axis < 3U; ++axis) {
		const unsigned int shift = axis * 4U;
		const uint8_t previous_values[2] = {
			(uint8_t)((previous_rate >> shift) & SCH16T_DATA_COUNTER_NIBBLE_MASK),
			(uint8_t)((previous_accel >> shift) & SCH16T_DATA_COUNTER_NIBBLE_MASK),
		};
		const uint8_t current_values[2] = {
			(uint8_t)((current_rate >> shift) & SCH16T_DATA_COUNTER_NIBBLE_MASK),
			(uint8_t)((current_accel >> shift) & SCH16T_DATA_COUNTER_NIBBLE_MASK),
		};
		unsigned int sensor;

		for (sensor = 0; sensor < 2U; ++sensor) {
			vector->residues[sensor * 3U + axis] =
				(uint8_t)((current_values[sensor] - previous_values[sensor]) &
					  SCH16T_DATA_COUNTER_NIBBLE_MASK);
		}
	}

	windowed = sch16t_counter_epoch_window(elapsed_ns, &low, &high);
	resolved = windowed;
	for (channel = 0; channel < SCH16T_DATA_COUNTER_CHANNELS; ++channel) {
		uint32_t advance = vector->residues[channel];

		if (windowed &&
		    resolve_channel_advance(vector->residues[channel], low, high, &advance) != 1U)
			resolved = 0;
		vector->channel_total += advance;
		if (advance < smallest)
			smallest = advance;
		if (advance > largest)
			largest = advance;
	}
	vector->resolved = resolved;
	vector->channel_span = (uint8_t)(largest - smallest > 0xFFU ? 0xFFU : largest - smallest);

	if (vector->channel_total == 0U)
		return SCH16T_COUNTER_DUPLICATE;

	vector->epoch_delta =
		(vector->channel_total + SCH16T_DATA_COUNTER_CHANNELS / 2U) / SCH16T_DATA_COUNTER_CHANNELS;
	if (vector->epoch_delta == 0U)
		vector->epoch_delta = 1U;
	return vector->epoch_delta == 1U ? SCH16T_COUNTER_NEXT : SCH16T_COUNTER_GAP;
}

int sch16t_config_manifest_valid(void)
{
	static const uint64_t expected[SCH16T_CONFIG_WRITE_COUNT] = {
		UINT64_C(0x096800000016), UINT64_C(0x09A800000020), UINT64_C(0x09E8000000D7),
		UINT64_C(0x0A28001324C7), UINT64_C(0x0A6800132430), UINT64_C(0x0AA8000000F8),
		UINT64_C(0x0D68000001D3),
	};
	uint64_t commands[SCH16T_CONFIG_WRITE_COUNT];
	unsigned int index;

	sch16t_config_write_commands(commands);
	for (index = 0; index < SCH16T_CONFIG_WRITE_COUNT; ++index) {
		const uint8_t address = (uint8_t)((commands[index] >> 38U) & UINT64_C(0x3F));

		if (commands[index] != expected[index] || address == SCH16T_REG_CTRL_USER_IF ||
		    !sch16t_frame_crc_valid(commands[index]))
			return 0;
	}
	return sch16t_soft_reset_command() == SCH16T_SOFT_RESET_FRAME &&
	       sch16t_eoi_command() == UINT64_C(0x0D680000038D);
}
