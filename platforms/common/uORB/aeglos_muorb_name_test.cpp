/****************************************************************************
 *
 * HOST-ONLY unit test for uORBNameEncoding.hpp. This file is NOT part of
 * any PX4 build target (it is intentionally absent from CMakeLists.txt).
 *
 * Build and run on a development host:
 *   c++ -std=c++11 -Wall -Wextra -Werror -o /tmp/muorb_name_test \
 *       platforms/common/uORB/aeglos_muorb_name_test.cpp && /tmp/muorb_name_test
 *
 ****************************************************************************/

#include "uORBNameEncoding.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond) \
	do { \
		if (!(cond)) { \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
			failures++; \
		} \
	} while (0)

// Mirror of ORB_MULTI_MAX_INSTANCES on VOXL2 (non-CONSTRAINED_MEMORY builds)
static const unsigned kMaxInstances = 10;

using namespace uORB::NameEncoding;

static void test_encode()
{
	char buf[64];

	// Instance 0 always returns the original pointer, buffer untouched
	memset(buf, 'X', sizeof(buf));
	const char *name = "sensor_gyro";
	CHECK(encode_wire_topic_name(name, 0, buf, sizeof(buf)) == name);
	CHECK(buf[0] == 'X');

	// Allowlisted topics with instance > 0 get the suffix
	CHECK(strcmp(encode_wire_topic_name("sensor_gyro", 3, buf, sizeof(buf)), "sensor_gyro@3") == 0);
	CHECK(strcmp(encode_wire_topic_name("sensor_accel", 1, buf, sizeof(buf)), "sensor_accel@1") == 0);
	CHECK(strcmp(encode_wire_topic_name("sensor_gyro", 9, buf, sizeof(buf)), "sensor_gyro@9") == 0);
	CHECK(strcmp(encode_wire_topic_name("vehicle_imu", 1, buf, sizeof(buf)), "vehicle_imu@1") == 0);
	CHECK(strcmp(encode_wire_topic_name("vehicle_imu_status", 1, buf, sizeof(buf)), "vehicle_imu_status@1") == 0);

	// Non-allowlisted topics keep the bare name at any instance
	const char *mag = "sensor_mag";
	CHECK(encode_wire_topic_name(mag, 2, buf, sizeof(buf)) == mag);
	const char *dist = "distance_sensor";
	CHECK(encode_wire_topic_name(dist, 1, buf, sizeof(buf)) == dist);

	// Buffer too small: falls back to the bare name, never truncates
	char tiny[8];
	const char *gyro = "sensor_gyro";
	CHECK(encode_wire_topic_name(gyro, 1, tiny, sizeof(tiny)) == gyro);
}

static void test_decode_valid()
{
	char base[64];
	unsigned instance = 99;

	// Bare name (stock wire format) => instance 0
	CHECK(decode_wire_topic_name("sensor_gyro", base, sizeof(base), kMaxInstances, &instance));
	CHECK(strcmp(base, "sensor_gyro") == 0);
	CHECK(instance == 0);

	// Valid suffix
	CHECK(decode_wire_topic_name("sensor_gyro@3", base, sizeof(base), kMaxInstances, &instance));
	CHECK(strcmp(base, "sensor_gyro") == 0);
	CHECK(instance == 3);

	// "@0" decodes to instance 0 (sender never emits it, but it is well formed)
	CHECK(decode_wire_topic_name("sensor_accel@0", base, sizeof(base), kMaxInstances, &instance));
	CHECK(strcmp(base, "sensor_accel") == 0);
	CHECK(instance == 0);

	// Generic parser: works for any topic name, not just the allowlist
	CHECK(decode_wire_topic_name("vehicle_odometry@2", base, sizeof(base), kMaxInstances, &instance));
	CHECK(strcmp(base, "vehicle_odometry") == 0);
	CHECK(instance == 2);
}

static void test_decode_malformed_falls_back_to_bare()
{
	char base[64];
	unsigned instance;

	const char *malformed[] = {
		"sensor_gyro@",     // no digits
		"sensor_gyro@x",    // non-digit
		"sensor_gyro@1x",   // trailing non-digit
		"sensor_gyro@123",  // too many digits
		"sensor_gyro@10",   // out of range (max 10 => 0..9)
		"sensor_gyro@99",   // out of range
		"a@1@2",            // multiple separators
		"@1",               // separator at position 0
		"keepalive",        // special muorb name, no separator
		"slpi_debug",
		"CPULOAD",
		"aggregation",
	};

	for (unsigned i = 0; i < sizeof(malformed) / sizeof(malformed[0]); i++) {
		instance = 77;
		CHECK(decode_wire_topic_name(malformed[i], base, sizeof(base), kMaxInstances, &instance));
		CHECK(strcmp(base, malformed[i]) == 0);
		CHECK(instance == 0);
	}
}

static void test_round_trip()
{
	char wire[64];
	char base[64];
	unsigned instance;

	const char *topics[] = {"sensor_gyro", "sensor_accel"};

	for (unsigned t = 0; t < 2; t++) {
		for (unsigned i = 0; i < kMaxInstances; i++) {
			const char *encoded = encode_wire_topic_name(topics[t], i, wire, sizeof(wire));
			CHECK(decode_wire_topic_name(encoded, base, sizeof(base), kMaxInstances, &instance));
			CHECK(strcmp(base, topics[t]) == 0);
			CHECK(instance == i);
		}
	}
}

static void test_decode_overflow()
{
	char tiny[4];
	unsigned instance;

	// Base name does not fit => hard failure (only failure mode)
	CHECK(!decode_wire_topic_name("sensor_gyro@1", tiny, sizeof(tiny), kMaxInstances, &instance));
	CHECK(!decode_wire_topic_name("sensor_gyro", tiny, sizeof(tiny), kMaxInstances, &instance));

	// Exactly fitting base name is fine
	char just[12]; // "sensor_gyro" = 11 chars + NUL
	CHECK(decode_wire_topic_name("sensor_gyro@4", just, sizeof(just), kMaxInstances, &instance));
	CHECK(strcmp(just, "sensor_gyro") == 0);
	CHECK(instance == 4);
}

int main()
{
	test_encode();
	test_decode_valid();
	test_decode_malformed_falls_back_to_bare();
	test_round_trip();
	test_decode_overflow();

	if (failures == 0) {
		printf("aeglos_muorb_name_test: ALL TESTS PASSED\n");
		return 0;
	}

	printf("aeglos_muorb_name_test: %d FAILURES\n", failures);
	return 1;
}

