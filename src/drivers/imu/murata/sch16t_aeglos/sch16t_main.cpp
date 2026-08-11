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

#include <string.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/time.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>

void SCH16T::print_usage()
{
	PRINT_MODULE_USAGE_NAME("sch16t", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

/**
 * muORB instance ordering guard.
 *
 * The instance-aware muORB patch transmits instance 0 bare-named for wire
 * compatibility: apps-side instance 0 of sensor_accel/sensor_gyro must be the
 * bridged DSP ICM stream. If this driver advertised first it would take
 * instance 0 and its publications would cross bare-named, interleaving into
 * (and corrupting) the DSP's instance-0 stream. PX4Accelerometer/PX4Gyroscope
 * advertise at construction, so the wait must complete before the driver
 * object is even instantiated. Fail closed on timeout: never advertise.
 */
static int sch16t_wait_for_bridged_instance0()
{
	static constexpr hrt_abstime TIMEOUT_US = 30 * 1000000ULL;
	const hrt_abstime start = hrt_absolute_time();

	while (orb_exists(ORB_ID(sensor_gyro), 0) != PX4_OK ||
	       orb_exists(ORB_ID(sensor_accel), 0) != PX4_OK) {

		if (hrt_elapsed_time(&start) > TIMEOUT_US) {
			PX4_ERR("bridged DSP IMU instance 0 (sensor_accel/sensor_gyro) not present after 30 s - refusing to start");
			return -1;
		}

		px4_usleep(100000);
	}

	return 0;
}

extern "C" int sch16t_main(int argc, char *argv[])
{
	using ThisDriver = SCH16T;
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = SCH16T_DEFAULT_SPI_HZ;
	cli.spi_mode = SPIDEV_MODE0;
	// Default to the board-internal SPI bus so plain `sch16t start` resolves the
	// board table entry (bus 14, chip-select 0 on VOXL2); -b/-s/-c still override.
	cli.bus_option = I2CSPIBusOption::SPIInternal;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_IMU_DEVTYPE_SCH16T);

	if (!strcmp(verb, "start")) {
		if (sch16t_wait_for_bridged_instance0() != 0) {
			return -1;
		}

		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}

