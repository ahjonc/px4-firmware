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
 * @file Sch16tSpidev.hpp
 *
 * Raw Linux spidev transport for the SCH16T SafeSPI framing.
 *
 * The generic posix device::SPI backend issues one spi_ioc_transfer per
 * SPI_IOC_MESSAGE(1) ioctl and offers neither per-transfer cs_change control nor
 * batched 6-byte transfers, both of which the qualified SafeSPI operating point
 * requires (per-frame chip select, one ioctl per batch). This class owns
 * /dev/spidevB.C directly with the same raw-ioctl pattern the qualified userspace
 * acquisition path used: mode 0, MSB first, 8 bits/word, N x 6-byte transfers with
 * cs_change=1 on all but the last, single SPI_IOC_MESSAGE(N) per batch.
 *
 * Every mode/speed/bits write is read back and verified; mismatch fails closed.
 */

#pragma once

#include <stdint.h>

#include "sch16t_protocol.h"

class Sch16tSpidev
{
public:
	static constexpr unsigned MAX_BATCH_FRAMES = SCH16T_MAX_FRAME_COUNT;

	Sch16tSpidev() = default;
	~Sch16tSpidev() { close(); }

	// non-copyable: owns a file descriptor
	Sch16tSpidev(const Sch16tSpidev &) = delete;
	Sch16tSpidev &operator=(const Sch16tSpidev &) = delete;

	/**
	 * Open the spidev node and configure+verify mode 0 / MSB first / 8 bits/word
	 * at the given speed. Returns 0 on success, negative errno-style otherwise.
	 */
	int open(const char *path, uint32_t speed_hz);

	/**
	 * Change the transfer speed (write-then-readback verified).
	 * Returns 0 on success, negative otherwise.
	 */
	int set_speed(uint32_t speed_hz);

	/**
	 * Transfer a batch of 48-bit SafeSPI frames: count x 6-byte transfers,
	 * cs_change=1 on all but the last, one SPI_IOC_MESSAGE(count) ioctl.
	 * Responses are the frames clocked in during each transfer (out-of-frame
	 * pipeline semantics are the caller's contract). Returns 0 on success.
	 */
	int transfer_batch(const uint64_t *commands, unsigned count, uint64_t *responses);

	void close();

	bool is_open() const { return _fd >= 0; }
	uint32_t speed_hz() const { return _speed_hz; }

private:
	int configure_and_verify();

	int _fd{-1};
	uint32_t _speed_hz{0};
};

