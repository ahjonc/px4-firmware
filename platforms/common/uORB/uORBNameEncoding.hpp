/****************************************************************************
 *
 *   Copyright (c) 2026 Aeglos. All rights reserved.
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
 * @file uORBNameEncoding.hpp
 *
 * Encoding of uORB multi-topic instance numbers into the topic name strings
 * that cross the muORB (apps <-> SLPI) communicator channel.
 *
 * The muORB wire protocol identifies topics by bare name only, which pins
 * every remote topic to instance 0. To let selected multi-instance topics
 * cross the channel, instance N > 0 of an allowlisted topic is encoded as
 * "<name>@<N>". Instance 0 of every topic, and every instance of any topic
 * not in the allowlist, keeps the bare name so the wire format stays
 * bit-identical to stock behavior. No PX4 topic name contains '@'.
 *
 * This header is deliberately self-contained (libc only) so it can be
 * compiled and unit tested on a host without the PX4 include web. See
 * aeglos_muorb_name_test.cpp.
 */

#pragma once

#include <string.h>
#include <stdio.h>

namespace uORB
{
namespace NameEncoding
{

/** Separator between topic name and instance number on the muORB wire. */
static const char kInstanceSeparator = '@';

/**
 * Sending-side allowlist. Only these topics ever get an instance suffix,
 * so no other multi-instance topic changes wire behavior. The receiving
 * side parser (decode_wire_topic_name) is generic on purpose.
 *
 * sensor_gyro/sensor_accel: the raw dual-IMU streams (apps -> DSP).
 * vehicle_imu/vehicle_imu_status: the DSP publishes one instance per IMU;
 * without encoding, both DSP instances would interleave into the apps-side
 * instance 0 and corrupt the logger's record (alternating device_ids).
 */
static inline bool instance_encoding_enabled(const char *name)
{
	return (strcmp(name, "sensor_gyro") == 0) || (strcmp(name, "sensor_accel") == 0)
	       || (strcmp(name, "vehicle_imu") == 0) || (strcmp(name, "vehicle_imu_status") == 0);
}

/**
 * Encode a topic name for transmission over the muORB channel.
 *
 * For instance 0, for topics not in the allowlist, and if the encoded name
 * would not fit in buf, the original name pointer is returned and buf is
 * left untouched (bare-name stock behavior). Otherwise buf receives
 * "<name>@<instance>" and buf is returned.
 */
static inline const char *encode_wire_topic_name(const char *name, unsigned instance, char *buf, unsigned buf_len)
{
	if (instance == 0 || !instance_encoding_enabled(name)) {
		return name;
	}

	int len = snprintf(buf, buf_len, "%s%c%u", name, kInstanceSeparator, instance);

	if (len < 0 || (unsigned)len >= buf_len) {
		// Never emit a truncated topic string
		return name;
	}

	return buf;
}

/**
 * Decode a wire topic name into a base name and an instance number.
 *
 * A valid suffix is a single '@' (not at position 0) followed by 1-2 digits
 * whose value is < max_instances. Anything else (no '@', multiple '@',
 * empty/overlong/non-numeric digits, out-of-range value) is treated as a
 * bare name with instance 0, which is exactly the stock behavior.
 *
 * @return false only if the base name does not fit in base_buf.
 */
static inline bool decode_wire_topic_name(const char *wire_name, char *base_buf, unsigned base_buf_len,
		unsigned max_instances, unsigned *instance)
{
	*instance = 0;

	unsigned base_len = strlen(wire_name);
	const char *at = strchr(wire_name, kInstanceSeparator);

	if ((at != nullptr) && (at != wire_name) && (strchr(at + 1, kInstanceSeparator) == nullptr)) {
		const char *digits = at + 1;
		size_t n_digits = strlen(digits);

		if ((n_digits >= 1) && (n_digits <= 2)) {
			unsigned value = 0;
			bool all_digits = true;

			for (size_t i = 0; i < n_digits; i++) {
				if ((digits[i] < '0') || (digits[i] > '9')) {
					all_digits = false;
					break;
				}

				value = (value * 10) + (unsigned)(digits[i] - '0');
			}

			if (all_digits && (value < max_instances)) {
				*instance = value;
				base_len = (unsigned)(at - wire_name);
			}
		}
	}

	if (base_len >= base_buf_len) {
		return false;
	}

	memcpy(base_buf, wire_name, base_len);
	base_buf[base_len] = '\0';
	return true;
}

} // namespace NameEncoding
} // namespace uORB

