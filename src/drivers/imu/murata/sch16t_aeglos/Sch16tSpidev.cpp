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

#include "Sch16tSpidev.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/spi/spidev.h>
#include <linux/types.h>

#include <px4_platform_common/log.h>

namespace
{
constexpr uint8_t SCH16T_SPI_MODE = SPI_MODE_0;
constexpr uint8_t SCH16T_SPI_BITS_PER_WORD = 8;

// SPI_IOC_MESSAGE(N) is only well-formed for a compile-time constant N (it takes
// sizeof of a char[N * sizeof(spi_ioc_transfer)] type). The qualified SafeSPI
// batch shapes are a fixed set; enumerate them and fail closed on anything else.
unsigned long spi_ioc_message_for(unsigned count)
{
	static_assert(SCH16T_CONFIG_WRITE_COUNT == SCH16T_IDENTITY_FRAME_COUNT,
		      "batch shape table assumes config writes and identity share a size");
	static_assert(SCH16T_CAPTURE_FRAME_COUNT == SCH16T_STATUS_FRAME_COUNT,
		      "batch shape table assumes capture and status share a size");

	switch (count) {
	case 1:					// soft reset / EOI
		return SPI_IOC_MESSAGE(1);

	case SCH16T_IDENTITY_FRAME_COUNT:	// identity cycle, golden config writes (7)
		return SPI_IOC_MESSAGE(SCH16T_IDENTITY_FRAME_COUNT);

	case SCH16T_STATUS_FRAME_COUNT:		// status sweep, capture batch (11)
		return SPI_IOC_MESSAGE(SCH16T_STATUS_FRAME_COUNT);

	case SCH16T_CONFIG_READ_FRAME_COUNT:	// config readback (12)
		return SPI_IOC_MESSAGE(SCH16T_CONFIG_READ_FRAME_COUNT);

	default:
		return 0;
	}
}
} // namespace

int Sch16tSpidev::open(const char *path, uint32_t speed_hz)
{
	if (_fd >= 0) {
		return -EBUSY;
	}

	_fd = ::open(path, O_RDWR | O_CLOEXEC);

	if (_fd < 0) {
		PX4_ERR("open %s failed (%d)", path, errno);
		return -errno;
	}

	// spidev permits concurrent opens; two readers interleave into each other's
	// off-frame reply pipelines. The retired userspace collector took the same
	// advisory lock, so this both excludes a stale collector and is honored by it.
	if (::flock(_fd, LOCK_EX | LOCK_NB) != 0) {
		PX4_ERR("%s is locked by another process (%d) — refusing shared bus", path, errno);
		close();
		return -EBUSY;
	}

	_speed_hz = speed_hz;

	int ret = configure_and_verify();

	if (ret != 0) {
		close();
	}

	return ret;
}

int Sch16tSpidev::configure_and_verify()
{
	// Write-then-readback each setting; a silent mismatch here would invalidate
	// every qualified transfer that follows, so any disagreement fails closed.
	uint8_t mode = SCH16T_SPI_MODE;

	if (::ioctl(_fd, SPI_IOC_WR_MODE, &mode) < 0) {
		PX4_ERR("SPI_IOC_WR_MODE failed (%d)", errno);
		return -errno;
	}

	uint8_t mode_readback = 0xFF;

	if (::ioctl(_fd, SPI_IOC_RD_MODE, &mode_readback) < 0 || mode_readback != SCH16T_SPI_MODE) {
		PX4_ERR("SPI mode readback mismatch: wanted 0x%02x got 0x%02x", SCH16T_SPI_MODE, mode_readback);
		return -EIO;
	}

	// SPI_MODE_0 has SPI_LSB_FIRST clear: the mode readback above already pins MSB-first.
	uint8_t bits = SCH16T_SPI_BITS_PER_WORD;

	if (::ioctl(_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
		PX4_ERR("SPI_IOC_WR_BITS_PER_WORD failed (%d)", errno);
		return -errno;
	}

	uint8_t bits_readback = 0;

	if (::ioctl(_fd, SPI_IOC_RD_BITS_PER_WORD, &bits_readback) < 0 || bits_readback != SCH16T_SPI_BITS_PER_WORD) {
		PX4_ERR("SPI bits/word readback mismatch: wanted %u got %u", SCH16T_SPI_BITS_PER_WORD, bits_readback);
		return -EIO;
	}

	return set_speed(_speed_hz);
}

int Sch16tSpidev::set_speed(uint32_t speed_hz)
{
	if (_fd < 0) {
		return -EBADF;
	}

	if (::ioctl(_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) {
		PX4_ERR("SPI_IOC_WR_MAX_SPEED_HZ %u failed (%d)", speed_hz, errno);
		return -errno;
	}

	uint32_t speed_readback = 0;

	if (::ioctl(_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed_readback) < 0 || speed_readback != speed_hz) {
		PX4_ERR("SPI speed readback mismatch: wanted %u got %u", speed_hz, speed_readback);
		return -EIO;
	}

	_speed_hz = speed_hz;
	return 0;
}

int Sch16tSpidev::transfer_batch(const uint64_t *commands, unsigned count, uint64_t *responses)
{
	if (_fd < 0) {
		return -EBADF;
	}

	if (count == 0 || count > MAX_BATCH_FRAMES) {
		return -EINVAL;
	}

	uint8_t tx[MAX_BATCH_FRAMES * SCH16T_FRAME_BYTE_COUNT];
	uint8_t rx[MAX_BATCH_FRAMES * SCH16T_FRAME_BYTE_COUNT];
	struct spi_ioc_transfer transfers[MAX_BATCH_FRAMES];
	memset(transfers, 0, sizeof(transfers));
	memset(rx, 0, sizeof(rx));

	for (unsigned index = 0; index < count; ++index) {
		sch16t_frame_to_wire_bytes(commands[index], &tx[index * SCH16T_FRAME_BYTE_COUNT]);

		transfers[index].tx_buf = (uint64_t)(uintptr_t)&tx[index * SCH16T_FRAME_BYTE_COUNT];
		transfers[index].rx_buf = (uint64_t)(uintptr_t)&rx[index * SCH16T_FRAME_BYTE_COUNT];
		transfers[index].len = SCH16T_FRAME_BYTE_COUNT;
		transfers[index].speed_hz = _speed_hz;
		transfers[index].bits_per_word = SCH16T_SPI_BITS_PER_WORD;
		// Per-frame chip select: toggle CS between frames, deassert after the last.
		transfers[index].cs_change = (index + 1U < count) ? 1 : 0;
	}

	const unsigned long request = spi_ioc_message_for(count);

	if (request == 0) {
		return -EINVAL;
	}

	const int expected = (int)(count * SCH16T_FRAME_BYTE_COUNT);
	const int result = ::ioctl(_fd, request, transfers);

	if (result != expected) {
		return (result < 0) ? -errno : -EIO;
	}

	for (unsigned index = 0; index < count; ++index) {
		responses[index] = sch16t_frame_from_wire_bytes(&rx[index * SCH16T_FRAME_BYTE_COUNT]);
	}

	return 0;
}

void Sch16tSpidev::close()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

