/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
 * Multicopter position controller.
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "mc_pos_control_d3_main.h"
#include "mc_pos_control_d3.h"

#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <sched.h>

using namespace pos_control_d3;

MulticopterPositionControlD3 *g_control;

void task_main_trampoline(int argc, char *argv[]) {
	g_control->main_loop();
}

int start() {
	ASSERT(g_control->_control_task == -1);
	/* start the task */
	g_control->_control_task = task_spawn_cmd("mc_pos_control",
	SCHED_DEFAULT,
	SCHED_PRIORITY_MAX - 5, 2000, (main_t) &task_main_trampoline, nullptr);
	if (g_control->_control_task < 0) {
		warn("task start failed");
		return -errno;
	}
	return OK;
}

int mc_pos_control_d3_main(int argc, char *argv[]) {
	if (argc < 1) {
		errx(1, "usage: mc_pos_control_d3 {start|stop|status}");
	}
	if (!strcmp(argv[1], "start")) {
		if (g_control != nullptr) {
			errx(1, "already running");
		}
		g_control = new MulticopterPositionControlD3;
		if (g_control == nullptr) {
			errx(1, "alloc failed");
		}
		if (OK != start()) {
			delete g_control;
			g_control = nullptr;
			err(1, "start failed");
		}
		exit(0);
	}
	if (!strcmp(argv[1], "stop")) {
		if (g_control == nullptr) {
			errx(1, "not running");
		}
		delete g_control;
		g_control = nullptr;
		exit(0);
	}
	if (!strcmp(argv[1], "status")) {
		if (g_control) {
			errx(0, "running");
		}
		else {
			errx(1, "not running");
		}
	}
	warnx("unrecognized command");
	return 1;
}
