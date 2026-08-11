/* SPDX-License-Identifier: GPL-2.0-or-later OR BSD-2-Clause */
/*
 * Murata SCH16T SafeSPI protocol helpers — the one protocol the device speaks.
 *
 * Host-buildable and kernel-linkable, dual-licensed so a GPL kernel driver and a
 * BSD userspace collector can both link this single object. Divergent CRC or
 * decode between consumers would be a latent cross-consumer bug no audit could
 * catch, which is why this file is shared and byte-identical rather than copied.
 *
 * Every register address, frame layout, and configuration word here is pinned by
 * the golden-frame fixture in fixtures/golden_frames.json, which is transcribed
 * from live device evidence. Do not invent addresses here — capture evidence
 * first, then extend the fixture and this file together.
 */
#ifndef SCH16T_PROTOCOL_H
#define SCH16T_PROTOCOL_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stddef.h>
#include <stdint.h>
#ifndef UINT64_C
#include <inttypes.h>
#endif
#endif

/*
 * UINT64_C comes from <stdint.h> and therefore does not exist in a kernel
 * translation unit. Several constants below are defined in terms of it and are
 * expanded by kernel consumers, so the fallback belongs here rather than in
 * each .c file: sch16t_protocol.c carried a private copy, which left
 * SCH16T_DIRECT_READ_BUDGET_NS unbuildable from a kernel translation unit
 * (-Werror=implicit-function-declaration) even though the host harness — which
 * does have <stdint.h> — compiled it clean.
 */
#ifndef UINT64_C
#define UINT64_C(c) c##ULL
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
	SCH16T_REG_RATE_X2 = 0x0A,
	SCH16T_REG_RATE_Y2 = 0x0B,
	SCH16T_REG_RATE_Z2 = 0x0C,
	SCH16T_REG_ACC_X2 = 0x0D,
	SCH16T_REG_ACC_Y2 = 0x0E,
	SCH16T_REG_ACC_Z2 = 0x0F,
	SCH16T_REG_TEMP = 0x10,
	SCH16T_REG_RATE_STATUS1 = 0x11,
	SCH16T_REG_ACC_STATUS1 = 0x12,

	SCH16T_REG_STAT_SUM = 0x14,
	SCH16T_REG_STAT_SUM_SAT = 0x15,
	SCH16T_REG_STAT_COM = 0x16,
	SCH16T_REG_STAT_RATE_COM = 0x17,
	SCH16T_REG_STAT_RATE_X = 0x18,
	SCH16T_REG_STAT_RATE_Y = 0x19,
	SCH16T_REG_STAT_RATE_Z = 0x1A,
	SCH16T_REG_STAT_ACC_X = 0x1B,
	SCH16T_REG_STAT_ACC_Y = 0x1C,
	SCH16T_REG_STAT_ACC_Z = 0x1D,

	SCH16T_REG_CTRL_FILT_RATE = 0x25,
	SCH16T_REG_CTRL_FILT_ACC12 = 0x26,
	SCH16T_REG_CTRL_FILT_ACC3 = 0x27,
	SCH16T_REG_CTRL_RATE = 0x28,
	SCH16T_REG_CTRL_ACC12 = 0x29,
	SCH16T_REG_CTRL_ACC3 = 0x2A,
	SCH16T_REG_CTRL_USER_IF = 0x33,
	SCH16T_REG_CTRL_MODE = 0x35,
	SCH16T_REG_CTRL_RESET = 0x36,

	SCH16T_REG_ASIC_ID = 0x3B,
	SCH16T_REG_COMP_ID = 0x3C,
	SCH16T_REG_SN_ID1 = 0x3D,
	SCH16T_REG_SN_ID2 = 0x3E,
	SCH16T_REG_SN_ID3 = 0x3F,

	SCH16T_SPI_SOFT_RESET_VALUE = 0x000A,
	SCH16T_CTRL_MODE_EN_SENSOR = 0x0001,
	SCH16T_CTRL_MODE_EOI = 0x0003,
	SCH16T_CTRL_USER_IF_DRY_SYNC = 0x0020,
	SCH16T_EXPECTED_COMP_ID = 0x0023,
	SCH16T_EXPECTED_ASIC_ID = 0x0021,
	SCH16T_STATUS_NORMAL_VALUE = 0xFFFF,

	SCH16T_FRAME_ADDRESS_SHIFT = 38,
	SCH16T_FRAME_RW_BIT = 37,
	SCH16T_FRAME_FT_BIT = 35,
	SCH16T_FRAME_DATA_SHIFT = 8,

	SCH16T_FRAME_BYTE_COUNT = 6,
	SCH16T_IDENTITY_FRAME_COUNT = 7,
	SCH16T_CONFIG_WRITE_COUNT = 7,
	SCH16T_STATUS_FRAME_COUNT = 11,
	SCH16T_CONFIG_READ_FRAME_COUNT = 12,
	SCH16T_SAMPLE_FRAME_COUNT = 8,
	SCH16T_CAPTURE_FRAME_COUNT = 11,
	SCH16T_MEASUREMENT_CONFIG_COUNT = 6,
	SCH16T_MAX_FRAME_COUNT = SCH16T_CONFIG_READ_FRAME_COUNT,

	SCH16T_DATA_COUNTER_CHANNELS = 6,
	SCH16T_COUNTER_WRAP_SPAN = 16,
	SCH16T_COUNTER_CHANNEL_SKEW = 1,
	SCH16T_DATA_COUNTER_NIBBLE_MASK = 0x0F,

	/* Qualified polling config: 68 Hz LPF0, channel-2 @ ~738 Hz, max K01 range. */
	SCH16T_CFG_FILT_RATE = 0x0000,
	SCH16T_CFG_FILT_ACC12 = 0x0000,
	SCH16T_CFG_FILT_ACC3 = 0x0000,
	SCH16T_CFG_CTRL_RATE = 0x1324,
	SCH16T_CFG_CTRL_ACC12 = 0x1324,
	SCH16T_CFG_CTRL_ACC3 = 0x0000,

	SCH16T_NOMINAL_ODR_HZ = 738,
	SCH16T_DEFAULT_SPI_HZ = 5000000,
	SCH16T_BRINGUP_SPI_HZ = 1000000,
	SCH16T_RESET_WAIT_MS = 300,
	SCH16T_EN_SENSOR_SETTLE_MS = 250,
	SCH16T_EOI_SETTLE_MS = 5,
	/*
	 * Duplicate-epoch retry budgets, in units of SCH16T_DUPLICATE_RETRY_DELAY_NS.
	 * The retry loop busy-waits with udelay(), so the right budget depends
	 * entirely on whether the caller owns a deadline.
	 *
	 * These count ITERATIONS, not microseconds. Each iteration also re-issues
	 * the whole eleven-frame capture batch, which costs several hundred
	 * microseconds on its own, so the udelay total badly understates the real
	 * cost of a retry. Callers that own a deadline must additionally pass a
	 * wall-clock bound to sch16t_read_capture_sample().
	 *
	 * PERIODIC: the counter-informed poll loop (hrtimer trigger, transport
	 * benchmark) is phase-locked to the device with a 25 us early-poll lead, so
	 * a duplicate normally clears in one retry. Busy-waiting past the slot
	 * boundary turns a recoverable duplicate into a missed slot, so this path
	 * passes its own poll period as the wall-clock bound and the iteration
	 * count is only a secondary cap.
	 *
	 * DIRECT: an unpaced read_raw() has no deadline of its own and simply wants
	 * the next sample. Its budget must span one whole device epoch (1275-1448 us
	 * over the F_PRIM envelope), otherwise a read issued shortly after another
	 * one fails closed with -EIO purely as a function of epoch phase — which is
	 * what reading the seven raw sysfs channels one file at a time does. It is
	 * bounded by SCH16T_DIRECT_READ_BUDGET_NS. Both budgets stay bounded in
	 * both dimensions, so a genuinely frozen counter still fails closed.
	 */
	SCH16T_DUPLICATE_EPOCH_RETRIES = 4,
	SCH16T_DUPLICATE_EPOCH_RETRIES_DIRECT = 10,
};

#define SCH16T_DATA_COUNTER_RESERVED_MASK 0xF000U
#define SCH16T_FRAME_GENERAL_ERROR UINT64_C(0x001000000000)
#define SCH16T_FRAME_COMMAND_ERROR UINT64_C(0x000800000000)
#define SCH16T_FRAME_DOING_INITIALIZATION UINT64_C(0x000600000000)
#define SCH16T_SOFT_RESET_FRAME UINT64_C(0x0DA800000AC3)

/* K01 max-range sensitivities used by bringup/capture (datasheet-backed). */
#define SCH16T_GYRO_LSB_PER_DPS 1600
#define SCH16T_ACCEL_LSB_PER_MPS2 3200
#define SCH16T_TEMP_LSB_PER_C 100

/* F_PRIM 22.1..25.1 kHz / decimation 32 → device epoch envelope (milli-Hz). */
#define SCH16T_DEVICE_ODR_MIN_MILLIHZ UINT64_C(690625)
#define SCH16T_DEVICE_ODR_MAX_MILLIHZ UINT64_C(784375)
#define SCH16T_COUNTER_EPOCH_SCALE UINT64_C(1000000000000)
#define SCH16T_COUNTER_MAX_INTERVAL_NS UINT64_C(10000000000)
#define SCH16T_DUPLICATE_RETRY_DELAY_NS 150000L
/*
 * Wall-clock bound for the unpaced read_raw() retry loop. The iteration budget
 * alone does not bound the time: each retry re-issues the whole capture batch,
 * so ten retries is ~10 x (batch + 150 us), not 1500 us. 3 ms still outlasts
 * the slowest device epoch the F_PRIM envelope permits (1447965 ns) by more
 * than a factor of two, so a read still cannot fail closed on epoch phase.
 */
#define SCH16T_DIRECT_READ_BUDGET_NS UINT64_C(3000000)
#define SCH16T_EPOCH_CARRY_DEBT_LIMIT (-2 * SCH16T_DATA_COUNTER_CHANNELS)

/**
 * struct sch16t_identity - ASIC / COMP / serial words from the out-of-frame pipeline
 * @comp_id: COMP_ID response word
 * @asic_id: ASIC_ID response word
 * @serial_id1: SN_ID1
 * @serial_id2: SN_ID2
 * @serial_id3: SN_ID3
 */
struct sch16t_identity {
	uint16_t comp_id;
	uint16_t asic_id;
	uint16_t serial_id1;
	uint16_t serial_id2;
	uint16_t serial_id3;
};

/**
 * struct sch16t_sample - decoded channel-2 inertial + temperature sample
 * @gyro: signed 20-bit rate LSBs (X/Y/Z)
 * @accel: signed 20-bit acceleration LSBs (X/Y/Z)
 * @temperature: temperature after datasheet >>4 (0.01 °C / LSB)
 */
struct sch16t_sample {
	int32_t gyro[3];
	int32_t accel[3];
	int32_t temperature;
};

/**
 * struct sch16t_data_counter_bracket - RATE/ACC STATUS1 counters around a batch
 * @rate_start: rate counter before inertial reads
 * @accel_start: accel counter before inertial reads
 * @rate_end: rate counter after inertial reads
 * @accel_end: accel counter after inertial reads
 */
struct sch16t_data_counter_bracket {
	uint16_t rate_start;
	uint16_t accel_start;
	uint16_t rate_end;
	uint16_t accel_end;
};

/**
 * struct sch16t_proto_validation - fail-closed decode diagnostics
 * @crc_errors: frames whose CRC8 did not match
 * @value_errors: unexpected identity/status/counter reserved bits
 * @status_errors: SafeSPI general/command/init status bits set
 */
struct sch16t_proto_validation {
	unsigned int crc_errors;
	unsigned int value_errors;
	unsigned int status_errors;
};

enum sch16t_counter_step {
	SCH16T_COUNTER_DUPLICATE = 0,
	SCH16T_COUNTER_NEXT = 1,
	SCH16T_COUNTER_GAP = 2,
};

/**
 * struct sch16t_counter_vector - six-channel wrap-resolved advance
 * @residues: per-channel nibble residues before wrap resolution
 * @channel_total: sum of wrap-resolved advances (load-bearing)
 * @epoch_delta: channel_total/6 rounded; classification only
 * @channel_span: max-min channel advance
 * @resolved: 1 when host interval selected exactly one wrap branch per channel
 */
struct sch16t_counter_vector {
	uint8_t residues[SCH16T_DATA_COUNTER_CHANNELS];
	uint32_t channel_total;
	uint32_t epoch_delta;
	uint8_t channel_span;
	int resolved;
};

uint8_t sch16t_crc8(uint64_t frame);
uint64_t sch16t_make_read_frame(uint8_t address);
uint64_t sch16t_make_write_frame(uint8_t address, uint16_t value);
int sch16t_frame_crc_valid(uint64_t frame);
uint16_t sch16t_frame_data16(uint64_t frame);
int32_t sch16t_frame_data20(uint64_t frame);

void sch16t_frame_to_wire_bytes(uint64_t frame, uint8_t output[SCH16T_FRAME_BYTE_COUNT]);
uint64_t sch16t_frame_from_wire_bytes(const uint8_t input[SCH16T_FRAME_BYTE_COUNT]);

void sch16t_identity_commands(uint64_t output[SCH16T_IDENTITY_FRAME_COUNT]);
uint64_t sch16t_soft_reset_command(void);
uint64_t sch16t_eoi_command(void);
void sch16t_config_write_commands(uint64_t output[SCH16T_CONFIG_WRITE_COUNT]);
void sch16t_status_read_commands(uint64_t output[SCH16T_STATUS_FRAME_COUNT]);
void sch16t_config_read_commands(uint64_t output[SCH16T_CONFIG_READ_FRAME_COUNT]);
void sch16t_sample_read_commands(uint64_t output[SCH16T_SAMPLE_FRAME_COUNT]);
void sch16t_capture_read_commands(uint64_t output[SCH16T_CAPTURE_FRAME_COUNT]);

int sch16t_validate_identity_cycle(const uint64_t responses[SCH16T_IDENTITY_FRAME_COUNT],
				   struct sch16t_identity *identity,
				   struct sch16t_proto_validation *validation);
int sch16t_validate_status_responses(const uint64_t responses[SCH16T_STATUS_FRAME_COUNT],
				     struct sch16t_proto_validation *validation);
int sch16t_validate_config_responses(const uint64_t responses[SCH16T_CONFIG_READ_FRAME_COUNT],
				     struct sch16t_proto_validation *validation);
int sch16t_decode_sample_responses(const uint64_t responses[SCH16T_SAMPLE_FRAME_COUNT],
				   struct sch16t_sample *sample,
				   struct sch16t_proto_validation *validation);
int sch16t_decode_bracketed_data_counter(const uint64_t responses[SCH16T_CAPTURE_FRAME_COUNT],
					 struct sch16t_data_counter_bracket *counters,
					 struct sch16t_proto_validation *validation);

/**
 * sch16t_reorder_capture_to_sample() - map 11-frame capture batch to 8-frame sample ABI
 *
 * Fast-path capture replies are out-of-frame: response[0] is rolling temperature from the
 * previous batch; responses[3..8] hold the current gyro/accel words. Pinned by the
 * capture-batch entry of the golden-frame fixture.
 */
void sch16t_reorder_capture_to_sample(const uint64_t capture[SCH16T_CAPTURE_FRAME_COUNT],
				      uint64_t sample[SCH16T_SAMPLE_FRAME_COUNT]);

int sch16t_counter_epoch_window(uint64_t elapsed_ns, uint64_t *low, uint64_t *high);
enum sch16t_counter_step sch16t_counter_vector_step(uint16_t previous_rate,
						    uint16_t previous_accel,
						    uint16_t current_rate,
						    uint16_t current_accel,
						    uint64_t elapsed_ns,
						    struct sch16t_counter_vector *vector);

/**
 * sch16t_config_manifest_valid() - fail-closed check of golden config write frames
 *
 * Returns 1 when soft-reset / EOI / seven config writes match the qualified bringup
 * manifest (including CRCs). Used at probe before talking to hardware.
 */
int sch16t_config_manifest_valid(void);

#ifdef __cplusplus
}
#endif

#endif /* SCH16T_PROTOCOL_H */

