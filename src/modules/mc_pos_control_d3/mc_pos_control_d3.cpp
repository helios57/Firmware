/*
 * mc_pos_control_d3.cpp
 *
 *  Created on: Dec 5, 2014
 *      Author: helios
 */

#include "mc_pos_control_d3.h"

#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>

using namespace pos_control_d3;

UOrbBridge::UOrbBridge() :
		vehicle_attitude_setpoint_publication(-1),
		updated(false) {
	vehicle_attitude_subscription = orb_subscribe(ORB_ID(vehicle_attitude));
	vehicle_attitude_setpoint_subscription = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	vehicle_control_mode_subscription = orb_subscribe(ORB_ID(vehicle_control_mode));
	manual_control_setpoint_subscription = orb_subscribe(ORB_ID(manual_control_setpoint));
	actuator_armed_subscription = orb_subscribe(ORB_ID(actuator_armed));
	d3_target_subscription = orb_subscribe(ORB_ID(d3_target));
	memset(&vehicle_attitude, 0, sizeof(vehicle_attitude));
	memset(&vehicle_attitude_setpoint, 0, sizeof(vehicle_attitude_setpoint));
	memset(&vehicle_control_mode, 0, sizeof(vehicle_control_mode));
	memset(&manual_control_setpoint, 0, sizeof(manual_control_setpoint));
	memset(&actuator_armed, 0, sizeof(actuator_armed));
	memset(&d3_target, 0, sizeof(d3_target));
}

UOrbBridge::~UOrbBridge() {
	orb_unsubscribe(vehicle_attitude_subscription);
	orb_unsubscribe(vehicle_attitude_setpoint_subscription);
	orb_unsubscribe(vehicle_control_mode_subscription);
	orb_unsubscribe(manual_control_setpoint_subscription);
	orb_unsubscribe(actuator_armed_subscription);
}

int UOrbBridge::getVehicleAttitudeSubscription() {
	return vehicle_attitude_subscription;
}
bool UOrbBridge::needsUpdate(int attSub) {
	orb_check(attSub, &updated);
	return updated;
}

void UOrbBridge::update() {
	if (needsUpdate(vehicle_attitude_subscription)) {
		orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_subscription, &vehicle_attitude);
	}
	if (needsUpdate(vehicle_attitude_setpoint_subscription)) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_subscription, &vehicle_attitude_setpoint);
	}
	if (needsUpdate(vehicle_control_mode_subscription)) {
		orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_subscription, &vehicle_control_mode);
	}
	if (needsUpdate(manual_control_setpoint_subscription)) {
		orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_subscription, &manual_control_setpoint);
	}
	if (needsUpdate(actuator_armed_subscription)) {
		orb_copy(ORB_ID(actuator_armed), actuator_armed_subscription, &actuator_armed);
	}
	if (needsUpdate(d3_target_subscription)) {
		orb_copy(ORB_ID(actuator_armed), d3_target_subscription, &d3_target);
	}
}

void UOrbBridge::publish() {
	if (vehicle_attitude_setpoint_publication > 0) {
		orb_publish(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_publication, &vehicle_attitude_setpoint);
	}
	else {
		vehicle_attitude_setpoint_publication = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &vehicle_attitude_setpoint);
	}
}

MulticopterPositionControlD3::MulticopterPositionControlD3() :
		_control_task(-1),
		_task_should_exit(false),
		_mavlink_fd(-1),
		uorb(nullptr) {
}

MulticopterPositionControlD3::~MulticopterPositionControlD3() {
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;
		/* wait for a second for the task to quit at our request */
		unsigned i = 0;
		do {
			/* wait 20ms */
			usleep(20000);
			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		}
		while (_control_task != -1);
	}
}

void MulticopterPositionControlD3::initialize() {
	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	uorb = new UOrbBridge();
	uorb->update();
	state.armed = false;
	state.fds[0].fd = uorb->getVehicleAttitudeSubscription();
	state.fds[0].events = POLLIN;
}

void MulticopterPositionControlD3::doLoop() {
	uorb->update();
	hrt_abstime currrentTimestamp = hrt_absolute_time();
	float dt = state.lastTimestamp != 0 ? (currrentTimestamp - state.lastTimestamp) * 0.000001f : 0.0f;
	state.lastTimestamp = currrentTimestamp;
	if (uorb->vehicle_control_mode.flag_armed && !state.armed) {
		/* reset setpoints and integrals on arming */
	}
	state.armed = uorb->vehicle_control_mode.flag_armed;
	if (uorb->vehicle_control_mode.flag_control_altitude_enabled || //
			uorb->vehicle_control_mode.flag_control_position_enabled || //
			uorb->vehicle_control_mode.flag_control_climb_rate_enabled || //
			uorb->vehicle_control_mode.flag_control_velocity_enabled) {
		state.manualXYinput = fabsf(uorb->manual_control_setpoint.x) > 0.01f || fabsf(uorb->manual_control_setpoint.y) > 0.01f;
		/* select control source */
		if (uorb->vehicle_control_mode.flag_control_manual_enabled) {
		}
	}
	if (dt < 0.001f) ;
}

void MulticopterPositionControlD3::main_loop() {
	initialize();
	while (!_task_should_exit) {
		/* wait for up to 500ms for data */
		int pret = poll(&state.fds[0], (sizeof(state.fds) / sizeof(state.fds[0])), 500);
		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}
		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}
		doLoop();
	}
	warnx("stopped");
	mavlink_log_info(_mavlink_fd, "[mpc] stopped");
	delete (uorb);
	_control_task = -1;
	_exit(0);
}

