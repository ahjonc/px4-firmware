############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# sch16t_protocol.c is the qualified SafeSPI protocol core, linked verbatim and
# compiled as C (px4_add_module handles mixed C/C++ sources).
px4_add_module(
	MODULE drivers__imu__murata__sch16t_aeglos
	MAIN sch16t
	COMPILE_FLAGS
	SRCS
		SCH16T.cpp
		SCH16T.hpp
		Sch16tSpidev.cpp
		Sch16tSpidev.hpp
		sch16t_main.cpp
		sch16t_protocol.c
		sch16t_protocol.h
		sch16t_px4_logic.h
	DEPENDS
		drivers_accelerometer
		drivers_gyroscope
		px4_work_queue
	)

# The posix platform compiles C sources with -D__KERNEL__; the dual-licensed
# protocol core keys its include selection on that macro. Undefine it for the
# verbatim file instead of editing it (byte-identity with the qualified source
# is load-bearing).
set_source_files_properties(sch16t_protocol.c PROPERTIES COMPILE_OPTIONS "-U__KERNEL__")

