/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * SCH16T PX4 driver logic that is pure computation: epoch pacing classification,
 * saturation flag extraction, SI scale factors, and the sensor-to-body rotation.
 *
 * Deliberately free of PX4 headers so the host conformance harness
 * (tests/test_sch16t_host.c) compiles this byte-for-byte with plain cc and pins
 * it against fixtures/golden_frames.json. The driver (SCH16T.cpp) includes the
 * same header; there is exactly one implementation of each rule.
 */

#ifndef SCH16T_PX4_LOGIC_H
#define SCH16T_PX4_LOGIC_H

#include <math.h>
#include <stdint.h>

#include "sch16t_protocol.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Saturation flag: the SafeSPI response status field S1:S0 occupies the two bits of
 * SCH16T_FRAME_DOING_INITIALIZATION (00 normal, 01 error, 10 saturation, 11 still
 * initializing). Saturation is the HIGH bit alone; both bits set means the sample is
 * from an initializing sensor and is rejected by sch16t_decode_sample_responses().
 * Derived from the protocol-core constant rather than retyped.
 */
#define SCH16T_PX4_FRAME_SATURATION \
	(SCH16T_FRAME_DOING_INITIALIZATION & ~(SCH16T_FRAME_DOING_INITIALIZATION >> 1))

static inline int sch16t_px4_frame_saturated(uint64_t response)
{
	return (response & SCH16T_FRAME_DOING_INITIALIZATION) == SCH16T_PX4_FRAME_SATURATION;
}

/*
 * Per-sensor saturation over the reordered 8-frame sample ABI
 * (sch16t_reorder_capture_to_sample): frames 1..3 gyro X/Y/Z, 4..6 accel X/Y/Z.
 */
static inline void sch16t_px4_saturation_flags(const uint64_t sample_responses[SCH16T_SAMPLE_FRAME_COUNT],
					       int *gyro_saturated, int *accel_saturated)
{
	unsigned int index;

	*gyro_saturated = 0;
	*accel_saturated = 0;

	for (index = 1; index <= 3; ++index) {
		if (sch16t_px4_frame_saturated(sample_responses[index])) {
			*gyro_saturated = 1;
		}
	}

	for (index = 4; index <= 6; ++index) {
		if (sch16t_px4_frame_saturated(sample_responses[index])) {
			*accel_saturated = 1;
		}
	}
}

/*
 * SI scaling at the qualified K01 max-range operating point, expressed from the
 * protocol-core sensitivities (datasheet-backed, pinned by the golden fixture's
 * scale_contract): gyro rad/s per LSB, accel m/s^2 per LSB, temperature C per LSB
 * after the core's >>4.
 */
#define SCH16T_PX4_GYRO_RAD_PER_LSB (M_PI / 180.0 / (double)SCH16T_GYRO_LSB_PER_DPS)
#define SCH16T_PX4_ACCEL_MPS2_PER_LSB (1.0 / (double)SCH16T_ACCEL_LSB_PER_MPS2)
#define SCH16T_PX4_TEMP_C_PER_LSB (1.0 / (double)SCH16T_TEMP_LSB_PER_C)

/* Qualified measurement ranges: +/-300 deg/s (RATE_RANGE_300), +/-80 m/s^2 (ACC12_RANGE_80). */
#define SCH16T_PX4_GYRO_RANGE_RAD (300.0 * M_PI / 180.0)
#define SCH16T_PX4_ACCEL_RANGE_MPS2 80.0

/*
 * Sensor axes expressed in the vehicle body frame (FRD): body = M * sensor.
 * Compile-time constant transcribed at full double precision from the calibration
 * record vehicles/starling/sensors/sch16t/core/calibration/sch16t_calibration.yaml
 * (rotation.matrix; provenance: paired capture 0a43ec4c-85ea-48e3-a68b-8bcf851f85d7,
 * gravity-direction disagreement 0.212 deg against the factory reference).
 * Applied exactly once, here; the PX4 rotation enum stays ROTATION_NONE.
 */
static const double sch16t_px4_sensor_to_body[3][3] = {
	{ -0.9998629200632482, 0.0038887414588795856, -0.01609406016081815 },
	{ 0.003668352160986702, 0.9998994110098316, 0.0137007683950116 },
	{ 0.01614572002163514, 0.013639851614176635, -0.9997766101349472 },
};

static inline void sch16t_px4_apply_sensor_to_body(const int32_t raw[3], float out[3])
{
	unsigned int row;

	for (row = 0; row < 3; ++row) {
		out[row] = (float)(sch16t_px4_sensor_to_body[row][0] * (double)raw[0] +
				   sch16t_px4_sensor_to_body[row][1] * (double)raw[1] +
				   sch16t_px4_sensor_to_body[row][2] * (double)raw[2]);
	}
}

/*
 * Exact payload equality across the six inertial channels.  This is diagnostic,
 * not a timing source: unlike the data-counter registers these values are the
 * measurements PX4 would actually consume.  Equality across all six signed
 * 20-bit values is a strong indication that an over-poll repeated one held
 * device epoch, while a changed value proves that at least one channel updated.
 */
static inline int sch16t_px4_inertial_payload_equal(const struct sch16t_sample *previous,
		const struct sch16t_sample *current)
{
	unsigned int axis;

	if (previous == NULL || current == NULL) {
		return 0;
	}

	for (axis = 0; axis < 3; ++axis) {
		if (previous->gyro[axis] != current->gyro[axis] ||
		    previous->accel[axis] != current->accel[axis]) {
			return 0;
		}
	}

	return 1;
}

/*
 * Pacing decision from bracketed data counters: DUPLICATE means the device is still
 * on the same decimation epoch (the normal ~8% over-poll case at the fixed 1240 us
 * host interval) and the sample must not be published; NEXT publishes; GAP publishes
 * and is accounted. Elapsed time is host time between the two compared brackets
 * (batch midpoints; both brackets read at the same in-batch phase).
 */
enum sch16t_px4_pacing {
	SCH16T_PX4_PACING_SKIP = 0,
	SCH16T_PX4_PACING_PUBLISH = 1,
	SCH16T_PX4_PACING_PUBLISH_GAP = 2,
};

static inline enum sch16t_px4_pacing sch16t_px4_classify_bracket(
	const struct sch16t_data_counter_bracket *previous,
	const struct sch16t_data_counter_bracket *current,
	uint64_t elapsed_ns,
	struct sch16t_counter_vector *vector)
{
	switch (sch16t_counter_vector_step(previous->rate_end, previous->accel_end,
					   current->rate_end, current->accel_end,
					   elapsed_ns, vector)) {
	case SCH16T_COUNTER_DUPLICATE:
		return SCH16T_PX4_PACING_SKIP;

	case SCH16T_COUNTER_NEXT:
		return SCH16T_PX4_PACING_PUBLISH;

	case SCH16T_COUNTER_GAP:
	default:
		return SCH16T_PX4_PACING_PUBLISH_GAP;
	}
}

#ifdef __cplusplus
}
#endif

#endif /* SCH16T_PX4_LOGIC_H */
