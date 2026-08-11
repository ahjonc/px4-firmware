/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Host conformance harness for the PX4 SCH16T driver logic.
 *
 * Links the verbatim protocol core (sch16t_protocol.c) plus the driver's pure
 * logic header (sch16t_px4_logic.h) and pins both against the golden-frame
 * fixture transcribed from live-device evidence. Runs with plain cc on any
 * host; no PX4 build needed.
 *
 *   cc -std=gnu11 -Wall -Wextra -Werror -I.. -o test_sch16t_host \
 *      test_sch16t_host.c ../sch16t_protocol.c && ./test_sch16t_host [fixture.json]
 */

#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sch16t_protocol.h"
#include "sch16t_px4_logic.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, ...) \
	do { \
		++g_checks; \
		if (!(cond)) { \
			++g_failures; \
			printf("FAIL %s:%d: ", __func__, __LINE__); \
			printf(__VA_ARGS__); \
			printf("\n"); \
		} \
	} while (0)

/* ---- minimal fixture access (fixed, known-shape JSON) ----------------------- */

static char *load_file(const char *path)
{
	FILE *file = fopen(path, "rb");

	if (!file) {
		fprintf(stderr, "cannot open fixture %s\n", path);
		exit(2);
	}

	fseek(file, 0, SEEK_END);
	long size = ftell(file);
	fseek(file, 0, SEEK_SET);
	char *data = (char *)malloc((size_t)size + 1U);

	if (!data || fread(data, 1, (size_t)size, file) != (size_t)size) {
		fprintf(stderr, "cannot read fixture %s\n", path);
		exit(2);
	}

	data[size] = '\0';
	fclose(file);
	return data;
}

static const char *find_key(const char *json, const char *key)
{
	char pattern[128];
	snprintf(pattern, sizeof(pattern), "\"%s\"", key);
	const char *position = strstr(json, pattern);

	if (!position) {
		fprintf(stderr, "fixture key %s not found\n", key);
		exit(2);
	}

	return position + strlen(pattern);
}

/* Parse the next quoted string after `position` as hex (with or without 0x). */
static uint64_t next_hex(const char **position)
{
	const char *open = strchr(*position, '"');

	if (!open) {
		fprintf(stderr, "expected quoted hex value\n");
		exit(2);
	}

	const char *close = strchr(open + 1, '"');
	char buffer[32];
	size_t length = (size_t)(close - open - 1);

	if (!close || length == 0 || length >= sizeof(buffer)) {
		fprintf(stderr, "malformed hex value\n");
		exit(2);
	}

	memcpy(buffer, open + 1, length);
	buffer[length] = '\0';
	*position = close + 1;
	return strtoull(buffer, NULL, 16);
}

/* Collect the frames_hex array inside the section starting at `section`. */
static unsigned collect_frames(const char *section, uint64_t *frames, unsigned max_frames)
{
	const char *position = strstr(section, "\"frames_hex\"");

	if (!position) {
		fprintf(stderr, "frames_hex not found\n");
		exit(2);
	}

	position = strchr(position, '[');
	const char *end = strchr(position, ']');
	unsigned count = 0;

	while (count < max_frames) {
		const char *open = strchr(position, '"');

		if (!open || open > end) {
			break;
		}

		frames[count++] = next_hex(&position);
	}

	return count;
}

static long next_integer(const char *position)
{
	const char *colon = strchr(position, ':');

	if (!colon) {
		fprintf(stderr, "expected integer value\n");
		exit(2);
	}

	return strtol(colon + 1, NULL, 10);
}

/* ---- fixture-pinned frame construction -------------------------------------- */

static void test_config_manifest(void)
{
	CHECK(sch16t_config_manifest_valid() == 1, "config manifest gate failed");
}

static void test_identity_commands(const char *json)
{
	uint64_t expected[SCH16T_IDENTITY_FRAME_COUNT];
	unsigned count = collect_frames(find_key(json, "identity_read_commands"), expected,
					SCH16T_IDENTITY_FRAME_COUNT);
	CHECK(count == SCH16T_IDENTITY_FRAME_COUNT, "identity fixture has %u frames", count);

	uint64_t commands[SCH16T_IDENTITY_FRAME_COUNT];
	sch16t_identity_commands(commands);

	for (unsigned index = 0; index < SCH16T_IDENTITY_FRAME_COUNT; ++index) {
		CHECK(commands[index] == expected[index],
		      "identity command %u: 0x%012" PRIX64 " != golden 0x%012" PRIX64,
		      index, commands[index], expected[index]);
	}
}

static void test_reset_eoi_commands(const char *json)
{
	const char *sequences = find_key(json, "sequences");
	const char *position = strstr(sequences, "\"soft_reset_command_hex\"");
	position += strlen("\"soft_reset_command_hex\"");
	uint64_t golden_reset = next_hex(&position);

	position = strstr(sequences, "\"eoi_command_hex\"");
	position += strlen("\"eoi_command_hex\"");
	uint64_t golden_eoi = next_hex(&position);

	CHECK(sch16t_soft_reset_command() == golden_reset,
	      "soft reset 0x%012" PRIX64 " != golden 0x%012" PRIX64, sch16t_soft_reset_command(), golden_reset);
	CHECK(sch16t_eoi_command() == golden_eoi,
	      "EOI 0x%012" PRIX64 " != golden 0x%012" PRIX64, sch16t_eoi_command(), golden_eoi);
	CHECK(sch16t_frame_crc_valid(golden_reset), "golden reset CRC");
	CHECK(sch16t_frame_crc_valid(golden_eoi), "golden EOI CRC");
}

static void test_config_write_commands(const char *json)
{
	uint64_t expected[SCH16T_CONFIG_WRITE_COUNT];
	unsigned count = collect_frames(find_key(json, "flight_config_write_commands"), expected,
					SCH16T_CONFIG_WRITE_COUNT);
	CHECK(count == SCH16T_CONFIG_WRITE_COUNT, "config write fixture has %u frames", count);

	uint64_t commands[SCH16T_CONFIG_WRITE_COUNT];
	sch16t_config_write_commands(commands);

	for (unsigned index = 0; index < SCH16T_CONFIG_WRITE_COUNT; ++index) {
		CHECK(commands[index] == expected[index],
		      "config write %u: 0x%012" PRIX64 " != golden 0x%012" PRIX64,
		      index, commands[index], expected[index]);

		const uint8_t address = (uint8_t)((commands[index] >> SCH16T_FRAME_ADDRESS_SHIFT) & 0x3F);
		CHECK(address != SCH16T_REG_CTRL_USER_IF, "config write %u targets CTRL_USER_IF", index);
	}
}

static void test_crc_vectors(const char *json)
{
	const char *vectors = find_key(json, "crc_vectors");
	const char *position = strstr(vectors, "\"valid_hex\"");
	position += strlen("\"valid_hex\"");
	uint64_t valid = next_hex(&position);

	position = strstr(vectors, "\"corrupt_hex\"");
	position += strlen("\"corrupt_hex\"");
	uint64_t corrupt = next_hex(&position);

	CHECK(sch16t_frame_crc_valid(valid) == 1, "golden valid frame rejected");
	CHECK(sch16t_frame_crc_valid(corrupt) == 0, "single-bit corrupt frame accepted");
}

static void test_identity_response_cycle(const char *json)
{
	const char *section = find_key(json, "identity_response_cycle");
	uint64_t responses[SCH16T_IDENTITY_FRAME_COUNT];
	unsigned count = collect_frames(section, responses, SCH16T_IDENTITY_FRAME_COUNT);
	CHECK(count == SCH16T_IDENTITY_FRAME_COUNT, "identity response fixture has %u frames", count);

	struct sch16t_identity identity;
	struct sch16t_proto_validation validation;
	CHECK(sch16t_validate_identity_cycle(responses, &identity, &validation) == 1,
	      "identity cycle rejected (crc %u value %u)", validation.crc_errors, validation.value_errors);

	const char *expected = strstr(section, "\"expected\"");
	const char *position = strstr(expected, "\"comp_id\"");
	position += strlen("\"comp_id\"");
	CHECK(identity.comp_id == (uint16_t)next_hex(&position), "comp_id mismatch");
	position = strstr(expected, "\"asic_id\"");
	position += strlen("\"asic_id\"");
	CHECK(identity.asic_id == (uint16_t)next_hex(&position), "asic_id mismatch");
	position = strstr(expected, "\"serial_id1\"");
	position += strlen("\"serial_id1\"");
	CHECK(identity.serial_id1 == (uint16_t)next_hex(&position), "serial_id1 mismatch");
	position = strstr(expected, "\"serial_id2\"");
	position += strlen("\"serial_id2\"");
	CHECK(identity.serial_id2 == (uint16_t)next_hex(&position), "serial_id2 mismatch");
	position = strstr(expected, "\"serial_id3\"");
	position += strlen("\"serial_id3\"");
	CHECK(identity.serial_id3 == (uint16_t)next_hex(&position), "serial_id3 mismatch");

	CHECK(identity.comp_id == SCH16T_EXPECTED_COMP_ID, "COMP_ID != 0x0023");
	CHECK(identity.asic_id == SCH16T_EXPECTED_ASIC_ID, "ASIC_ID != 0x0021");
}

static void test_status_responses(const char *json)
{
	const char *section = find_key(json, "validated_status_responses");
	uint64_t golden[SCH16T_STATUS_FRAME_COUNT - 1];
	unsigned count = collect_frames(section, golden, SCH16T_STATUS_FRAME_COUNT - 1);
	CHECK(count == SCH16T_STATUS_FRAME_COUNT - 1, "status fixture has %u frames", count);

	/* Behind a zero priming frame, exactly as the acquisition path presents them. */
	uint64_t responses[SCH16T_STATUS_FRAME_COUNT] = { 0 };
	memcpy(&responses[1], golden, sizeof(golden));

	struct sch16t_proto_validation validation;
	CHECK(sch16t_validate_status_responses(responses, &validation) == 1,
	      "status sweep rejected (crc %u value %u)", validation.crc_errors, validation.value_errors);
}

/* ---- capture reorder mapping ------------------------------------------------- */

static void test_capture_reorder(void)
{
	uint64_t capture[SCH16T_CAPTURE_FRAME_COUNT];

	for (unsigned index = 0; index < SCH16T_CAPTURE_FRAME_COUNT; ++index) {
		capture[index] = 0x1000U + index;
	}

	uint64_t sample[SCH16T_SAMPLE_FRAME_COUNT];
	sch16t_reorder_capture_to_sample(capture, sample);

	CHECK(sample[0] == 0, "sample[0] must be the zero priming slot");

	for (unsigned index = 0; index < 6; ++index) {
		CHECK(sample[index + 1] == capture[index + 3],
		      "sample[%u] != capture[%u]", index + 1, index + 3);
	}

	CHECK(sample[7] == capture[0], "sample[7] (temp) != capture[0] (rolling temperature)");
}

/* ---- pacing classification ---------------------------------------------------- */

static void test_pacing_classification(void)
{
	struct sch16t_counter_vector vector;
	struct sch16t_data_counter_bracket previous = { 0x0555, 0x0555, 0x0555, 0x0555 };

	/* Same epoch, 1240 us later: the normal over-poll duplicate -> skip. */
	struct sch16t_data_counter_bracket duplicate = previous;
	CHECK(sch16t_px4_classify_bracket(&previous, &duplicate, 1240000ULL, &vector) == SCH16T_PX4_PACING_SKIP,
	      "duplicate epoch not classified as skip");

	/* Every channel advanced one epoch in ~1.35 ms -> publish. */
	struct sch16t_data_counter_bracket next = { 0x0666, 0x0666, 0x0666, 0x0666 };
	CHECK(sch16t_px4_classify_bracket(&previous, &next, 1348000ULL, &vector) == SCH16T_PX4_PACING_PUBLISH,
	      "single-epoch advance not classified as publish");
	CHECK(vector.epoch_delta == 1, "epoch_delta %u != 1", vector.epoch_delta);
	CHECK(vector.resolved == 1, "single-epoch advance unresolved");

	/* Two epochs advanced in ~2.7 ms (a missed epoch) -> publish + gap. */
	struct sch16t_data_counter_bracket gap = { 0x0777, 0x0777, 0x0777, 0x0777 };
	CHECK(sch16t_px4_classify_bracket(&previous, &gap, 2700000ULL, &vector) == SCH16T_PX4_PACING_PUBLISH_GAP,
	      "double-epoch advance not classified as gap");
	CHECK(vector.epoch_delta == 2, "epoch_delta %u != 2", vector.epoch_delta);

	/* Reserved counter bits (in the compared end counters) fail closed as gap. */
	struct sch16t_data_counter_bracket reserved = { 0x0555, 0x0555, 0xF555, 0x0555 };
	CHECK(sch16t_px4_classify_bracket(&reserved, &duplicate, 1240000ULL, &vector) == SCH16T_PX4_PACING_PUBLISH_GAP,
	      "reserved counter bits did not fail closed");
}

/* ---- sensor-to-body transform -------------------------------------------------- */

static void test_sensor_to_body_transform(void)
{
	const int32_t unit = 1000000;
	int32_t raw[3];
	float out[3];

	/* x -> -x */
	raw[0] = unit; raw[1] = 0; raw[2] = 0;
	sch16t_px4_apply_sensor_to_body(raw, out);
	CHECK(out[0] < -0.999f * (float)unit, "x -> -x: out[0]=%f", (double)out[0]);
	CHECK(fabsf(out[1]) < 0.02f * (float)unit && fabsf(out[2]) < 0.02f * (float)unit,
	      "x cross terms too large: %f %f", (double)out[1], (double)out[2]);

	/* y -> +y */
	raw[0] = 0; raw[1] = unit; raw[2] = 0;
	sch16t_px4_apply_sensor_to_body(raw, out);
	CHECK(out[1] > 0.999f * (float)unit, "y -> +y: out[1]=%f", (double)out[1]);
	CHECK(fabsf(out[0]) < 0.02f * (float)unit && fabsf(out[2]) < 0.02f * (float)unit,
	      "y cross terms too large: %f %f", (double)out[0], (double)out[2]);

	/* z -> -z */
	raw[0] = 0; raw[1] = 0; raw[2] = unit;
	sch16t_px4_apply_sensor_to_body(raw, out);
	CHECK(out[2] < -0.999f * (float)unit, "z -> -z: out[2]=%f", (double)out[2]);
	CHECK(fabsf(out[0]) < 0.02f * (float)unit && fabsf(out[1]) < 0.02f * (float)unit,
	      "z cross terms too large: %f %f", (double)out[0], (double)out[1]);

	/* Rotation must preserve magnitude (orthonormal to calibration precision). */
	raw[0] = 300000; raw[1] = -400000; raw[2] = 1200000;
	sch16t_px4_apply_sensor_to_body(raw, out);
	const double magnitude_in = sqrt((double)raw[0] * raw[0] + (double)raw[1] * raw[1] + (double)raw[2] * raw[2]);
	const double magnitude_out = sqrt((double)out[0] * out[0] + (double)out[1] * out[1] + (double)out[2] * out[2]);
	CHECK(fabs(magnitude_in - magnitude_out) / magnitude_in < 1e-3,
	      "transform not norm-preserving: %f vs %f", magnitude_in, magnitude_out);
}

/* ---- sample decode, saturation, scales ---------------------------------------- */

static uint64_t make_response(int32_t data20, uint64_t status_bits)
{
	uint64_t frame = ((uint64_t)((uint32_t)data20 & 0xFFFFFU) << SCH16T_FRAME_DATA_SHIFT) | status_bits;
	return frame | sch16t_crc8(frame);
}

static void test_sample_decode_and_saturation(void)
{
	uint64_t responses[SCH16T_SAMPLE_FRAME_COUNT];
	responses[0] = 0; /* priming slot, never inspected */
	responses[1] = make_response(-1600, 0);			/* gyro X: -1 deg/s */
	responses[2] = make_response(3200, SCH16T_PX4_FRAME_SATURATION); /* gyro Y saturated */
	responses[3] = make_response(0, 0);
	responses[4] = make_response(3200, 0);			/* accel X: +1 m/s^2 */
	responses[5] = make_response(-6400, 0);			/* accel Y: -2 m/s^2 */
	responses[6] = make_response(0, 0);
	responses[7] = make_response(40000, 0);			/* temp: 40000>>4 = 2500 -> 25.00 C */

	struct sch16t_sample sample;
	struct sch16t_proto_validation validation;
	CHECK(sch16t_decode_sample_responses(responses, &sample, &validation) == 1,
	      "decode failed (crc %u status %u)", validation.crc_errors, validation.status_errors);
	CHECK(sample.gyro[0] == -1600, "gyro X %d", sample.gyro[0]);
	CHECK(sample.gyro[1] == 3200, "gyro Y %d", sample.gyro[1]);
	CHECK(sample.accel[0] == 3200, "accel X %d", sample.accel[0]);
	CHECK(sample.accel[1] == -6400, "accel Y %d", sample.accel[1]);
	CHECK(sample.temperature == 2500, "temperature %d", sample.temperature);

	int gyro_saturated = 0;
	int accel_saturated = 0;
	sch16t_px4_saturation_flags(responses, &gyro_saturated, &accel_saturated);
	CHECK(gyro_saturated == 1 && accel_saturated == 0,
	      "saturation flags gyro %d accel %d", gyro_saturated, accel_saturated);

	/* SI conversion at the decoded values */
	const double gyro_x_rad = sample.gyro[0] * SCH16T_PX4_GYRO_RAD_PER_LSB;
	CHECK(fabs(gyro_x_rad + M_PI / 180.0) < 1e-12, "gyro scale: %.15f", gyro_x_rad);
	const double accel_y_mps2 = sample.accel[1] * SCH16T_PX4_ACCEL_MPS2_PER_LSB;
	CHECK(fabs(accel_y_mps2 + 2.0) < 1e-12, "accel scale: %.15f", accel_y_mps2);
	const double temp_c = sample.temperature * SCH16T_PX4_TEMP_C_PER_LSB;
	CHECK(fabs(temp_c - 25.0) < 1e-12, "temp scale: %.15f", temp_c);

	/* Both status bits set = still initializing = reject, not saturation. */
	responses[2] = make_response(3200, SCH16T_FRAME_DOING_INITIALIZATION);
	CHECK(sch16t_decode_sample_responses(responses, &sample, &validation) == 0,
	      "initializing sample not rejected");
	CHECK(sch16t_px4_frame_saturated(responses[2]) == 0, "init flagged as saturation");
}

static void test_scale_contract(const char *json)
{
	const char *contract = find_key(json, "scale_contract");
	CHECK(next_integer(strstr(contract, "\"gyro_lsb_per_dps\"")) == SCH16T_GYRO_LSB_PER_DPS,
	      "gyro sensitivity contract mismatch");
	CHECK(next_integer(strstr(contract, "\"accel_lsb_per_mps2\"")) == SCH16T_ACCEL_LSB_PER_MPS2,
	      "accel sensitivity contract mismatch");
	CHECK(next_integer(strstr(contract, "\"temp_lsb_per_c\"")) == SCH16T_TEMP_LSB_PER_C,
	      "temp sensitivity contract mismatch");

	CHECK(fabs(SCH16T_PX4_GYRO_RAD_PER_LSB - (M_PI / 180.0) / 1600.0) < 1e-18, "gyro rad/LSB");
	CHECK(fabs(SCH16T_PX4_ACCEL_MPS2_PER_LSB - 1.0 / 3200.0) < 1e-18, "accel m/s^2 per LSB");
	CHECK(fabs(SCH16T_PX4_GYRO_RANGE_RAD - 300.0 * M_PI / 180.0) < 1e-15, "gyro range");
	CHECK(SCH16T_PX4_ACCEL_RANGE_MPS2 == 80.0, "accel range");
}

int main(int argc, char *argv[])
{
	const char *fixture_path = (argc > 1) ? argv[1] : "../fixtures/golden_frames.json";
	char *json = load_file(fixture_path);

	test_config_manifest();
	test_identity_commands(json);
	test_reset_eoi_commands(json);
	test_config_write_commands(json);
	test_crc_vectors(json);
	test_identity_response_cycle(json);
	test_status_responses(json);
	test_capture_reorder();
	test_pacing_classification();
	test_sensor_to_body_transform();
	test_sample_decode_and_saturation();
	test_scale_contract(json);

	free(json);

	printf("%s: %d checks, %d failures\n", g_failures == 0 ? "PASS" : "FAIL", g_checks, g_failures);
	return g_failures == 0 ? 0 : 1;
}

