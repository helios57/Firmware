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
	sensor_combined_subscription = orb_subscribe(ORB_ID(sensor_combined));
	vehicle_gps_position_subscription = orb_subscribe(ORB_ID(vehicle_gps_position));
	optical_flow_subscription = orb_subscribe(ORB_ID(optical_flow));
	memset(&vehicle_attitude, 0, sizeof(vehicle_attitude));
	memset(&vehicle_attitude_setpoint, 0, sizeof(vehicle_attitude_setpoint));
	memset(&vehicle_control_mode, 0, sizeof(vehicle_control_mode));
	memset(&manual_control_setpoint, 0, sizeof(manual_control_setpoint));
	memset(&actuator_armed, 0, sizeof(actuator_armed));
	memset(&d3_target, 0, sizeof(d3_target));
	memset(&sensor_combined, 0, sizeof(sensor_combined));
	memset(&vehicle_gps_position, 0, sizeof(vehicle_gps_position));
	memset(&optical_flow, 0, sizeof(optical_flow));
}

UOrbBridge::~UOrbBridge() {
	orb_unsubscribe(vehicle_attitude_subscription);
	orb_unsubscribe(vehicle_attitude_setpoint_subscription);
	orb_unsubscribe(vehicle_control_mode_subscription);
	orb_unsubscribe(manual_control_setpoint_subscription);
	orb_unsubscribe(actuator_armed_subscription);
	orb_unsubscribe(d3_target_subscription);
	orb_unsubscribe(sensor_combined_subscription);
	orb_unsubscribe(vehicle_gps_position_subscription);
	orb_unsubscribe(optical_flow_subscription);
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
		orb_copy(ORB_ID(d3_target), d3_target_subscription, &d3_target);
	}
	if (needsUpdate(sensor_combined_subscription)) {
		orb_copy(ORB_ID(sensor_combined), sensor_combined_subscription, &sensor_combined);
	}
	if (needsUpdate(vehicle_gps_position_subscription)) {
		orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_subscription, &vehicle_gps_position);
	}
	if (needsUpdate(optical_flow_subscription)) {
		orb_copy(ORB_ID(optical_flow), optical_flow_subscription, &optical_flow);
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
	memset(&state, 0, sizeof(state));
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
	state.R.identity();
}

void MulticopterPositionControlD3::publishAttitudeSetpoint() {
	uorb->vehicle_attitude_setpoint.roll_body = state.rollSetpoint;
	uorb->vehicle_attitude_setpoint.pitch_body = state.pitchSetpoint;
	uorb->vehicle_attitude_setpoint.yaw_body = state.yawSetpoint;
	uorb->vehicle_attitude_setpoint.thrust = state.thrustSetpoint;
	state.R.from_euler(state.rollSetpoint, state.pitchSetpoint, state.yawSetpoint);
	memcpy(&uorb->vehicle_attitude_setpoint.R_body[0][0], state.R.data, sizeof(uorb->vehicle_attitude_setpoint.R_body));
	uorb->vehicle_attitude_setpoint.R_valid = true;
	uorb->vehicle_attitude_setpoint.timestamp = hrt_absolute_time();
	uorb->publish();
}

void MulticopterPositionControlD3::resetSetpointsIfNeeded() {
	if ((uorb->vehicle_control_mode.flag_armed && !state.armed) || (checkEnablement() && !state.enabled)) {
		state.rollSetpoint = 0.0f;
		state.pitchSetpoint = 0.0f;
		state.yawSetpoint = uorb->vehicle_attitude.yaw;
		state.groundDistLocalSP = state.groundDistLocal;
		state.thrustSetpoint = state.manualZ;
	}
	state.armed = uorb->vehicle_control_mode.flag_armed;
	state.enabled = checkEnablement();
}

bool MulticopterPositionControlD3::checkEnablement() {
	return uorb->vehicle_control_mode.flag_control_altitude_enabled || //
			uorb->vehicle_control_mode.flag_control_position_enabled || //
			uorb->vehicle_control_mode.flag_control_climb_rate_enabled || //
			uorb->vehicle_control_mode.flag_control_velocity_enabled;
}

void MulticopterPositionControlD3::applyRCInputIfAvailable(float dt) {
	state.manualX = uorb->manual_control_setpoint.x;
	state.manualY = uorb->manual_control_setpoint.y;
	state.manualZ = uorb->manual_control_setpoint.z;
	state.manualR = uorb->manual_control_setpoint.r;
	state.manualXYinput = !isnan(state.manualX) && !isnan(state.manualY) && !isnan(state.manualZ) && !isnan(state.manualR) && (fabsf(state.manualX) > 0.01f || fabsf(state.manualY) > 0.01f);
	if (uorb->vehicle_control_mode.flag_control_manual_enabled) {
		//state.thrustSetpoint = state.manualZ;
		float rcZ = state.manualZ - 0.5f;
		if (fabsf(rcZ) > 0.2f && fabsf(state.groundDistLocalSP - state.groundDistLocal) < 9.9f) {
			state.groundDistLocalSP += rcZ * dt * 0.5f;
		}
		state.yawSetpoint = _wrap_pi(state.yawSetpoint + state.manualR * dt);
		state.rollSetpoint = state.manualY * 0.6f;
		state.pitchSetpoint = -state.manualX * 0.6f;
		float diff = state.groundDistLocalSP - state.groundDistLocal;
		state.thrustSetpoint += min(diff * 0.1f, 0.1f);
		state.thrustSetpoint = max(min(state.thrustSetpoint, 0.9f), 0.0f);
	}
}

bool MulticopterPositionControlD3::newTargetDetected() {
	uint64_t timestampExternal = uorb->d3_target.timestamp;
	bool newTargetDetection = timestampExternal != state.targetLastTimestampExternal;
	state.targetLastTimestampExternal = timestampExternal;
	return newTargetDetection;
}

void MulticopterPositionControlD3::applyTargetInput(hrt_abstime currrentTimestamp) {
	if (!state.manualXYinput) {
		if (newTargetDetected()) {
			state.targetLastTimestampLocal = currrentTimestamp;
		}
		hrt_abstime targetAge = currrentTimestamp - state.targetLastTimestampLocal;
		//accept 1s old target
		if (targetAge < 1000000) {
			float q = ((1000000.0f - targetAge) / 1000000.0f) * 0.3f; //0-1.0
			float targetRadX = uorb->d3_target.x;
			float targetRadY = uorb->d3_target.y;
			state.rollSetpoint = -targetRadX * q;
			state.pitchSetpoint = -targetRadY * q;
		}
		else {
			state.rollSetpoint = 0.0f;
			state.pitchSetpoint = 0.0f;
		}
	}
}

float MulticopterPositionControlD3::filterGroundDist(float groundDistLocalOld, float groundDistSonarM) {
	float diff = max(fabsf(groundDistLocalOld - groundDistSonarM), 0.1f); //0.1 - 3.8
	float p = 5.0f / (diff * 100.0f);
	float newGroundDist = groundDistLocalOld * p + groundDistSonarM * (1 - p);
	return newGroundDist;
}

void MulticopterPositionControlD3::calculateGroundDistance() {
	float groundDistSonarM = uorb->optical_flow.ground_distance_m; //0.3 - 3.8 local
	float groundDistBaroM = uorb->sensor_combined.baro_alt_meter; // global
	float groundDistGPSM = uorb->vehicle_gps_position.alt / 1000.0f; // global
	bool sonarValid = groundDistSonarM > 0.3f && groundDistSonarM < 3.8f;
	bool gpsValid = uorb->vehicle_gps_position.fix_type >= 3;
	float groundDistLocalOld = state.groundDistLocal;
	//float groundDistMSLOld = state.groundDistMSL;
	if (gpsValid) {
		state.groundDistMSL = groundDistGPSM;
		state.groundDistBaroOffset = state.groundDistBaroOffset * 0.95f + (state.groundDistMSL - groundDistBaroM) * 0.05f;
	}
	else {
		state.groundDistMSL = groundDistBaroM + state.groundDistBaroOffset;
	}
	if (sonarValid) {
		state.groundDistLocal = filterGroundDist(groundDistLocalOld, groundDistSonarM);
		state.groundDistMSLOffset = state.groundDistMSLOffset * 0.95f + (state.groundDistLocal - state.groundDistMSL) * 0.05f;
	}
	else {
		state.groundDistLocal = filterGroundDist(groundDistLocalOld, state.groundDistMSL + state.groundDistMSLOffset);
	}
}

void MulticopterPositionControlD3::doLoop() {
	uorb->update();
	hrt_abstime currrentTimestamp = hrt_absolute_time();
	float dt = state.lastTimestamp != 0 ? (currrentTimestamp - state.lastTimestamp) * 0.000001f : 0.0f;
	state.lastTimestamp = currrentTimestamp;
	resetSetpointsIfNeeded();
	calculateGroundDistance();
	applyRCInputIfAvailable(dt);
	applyTargetInput(currrentTimestamp);

	if (checkEnablement()) {
		publishAttitudeSetpoint();
	}
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

