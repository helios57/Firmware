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

void UOrbBridge::updateParams(bool force) {
	orb_check(parameter_update_subscription, &updated);
	if (updated || force) {
		orb_copy(ORB_ID(parameter_update), parameter_update_subscription, &parameter_update);
		param_get(param_handles.thr_min, &params.thr_min);
		param_get(param_handles.thr_max, &params.thr_max);
		param_get(param_handles.tilt_max_air, &params.tilt_max_air);
		params.tilt_max_air = math::radians(params.tilt_max_air);
		param_get(param_handles.land_speed, &params.land_speed);
		param_get(param_handles.tilt_max_land, &params.tilt_max_land);
		params.tilt_max_land = math::radians(params.tilt_max_land);
		float f;
		param_get(param_handles.xy_p, &f);
		params.pos_p(0) = f;
		params.pos_p(1) = f;
		param_get(param_handles.z_p, &f);
		params.pos_p(2) = f;
		param_get(param_handles.xy_vel_p, &f);
		params.vel_p(0) = f;
		params.vel_p(1) = f;
		param_get(param_handles.z_vel_p, &f);
		params.vel_p(2) = f;
		param_get(param_handles.xy_vel_i, &f);
		params.vel_i(0) = f;
		params.vel_i(1) = f;
		param_get(param_handles.z_vel_i, &f);
		params.vel_i(2) = f;
		param_get(param_handles.xy_vel_d, &f);
		params.vel_d(0) = f;
		params.vel_d(1) = f;
		param_get(param_handles.z_vel_d, &f);
		params.vel_d(2) = f;
		param_get(param_handles.xy_vel_max, &f);
		params.vel_max(0) = f;
		params.vel_max(1) = f;
		param_get(param_handles.z_vel_max, &f);
		params.vel_max(2) = f;
		param_get(param_handles.xy_ff, &f);
		f = math::constrain(f, 0.0f, 1.0f);
		params.vel_ff(0) = f;
		params.vel_ff(1) = f;
		param_get(param_handles.z_ff, &f);
		f = math::constrain(f, 0.0f, 1.0f);
		params.vel_ff(2) = f;
		params.sp_offs_max = params.vel_max.edivide(params.pos_p) * 2.0f;
	}
}

UOrbBridge::UOrbBridge() :
		vehicle_attitude_setpoint_publication(-1),
		vehicle_local_position_setpoint_publication(-1),
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
	vehicle_local_position_subscription = orb_subscribe(ORB_ID(vehicle_local_position));
	parameter_update_subscription = orb_subscribe(ORB_ID(parameter_update));
	memset(&vehicle_attitude, 0, sizeof(vehicle_attitude));
	memset(&vehicle_attitude_setpoint, 0, sizeof(vehicle_attitude_setpoint));
	memset(&vehicle_control_mode, 0, sizeof(vehicle_control_mode));
	memset(&manual_control_setpoint, 0, sizeof(manual_control_setpoint));
	memset(&actuator_armed, 0, sizeof(actuator_armed));
	memset(&d3_target, 0, sizeof(d3_target));
	memset(&sensor_combined, 0, sizeof(sensor_combined));
	memset(&vehicle_gps_position, 0, sizeof(vehicle_gps_position));
	memset(&optical_flow, 0, sizeof(optical_flow));
	memset(&vehicle_local_position, 0, sizeof(vehicle_local_position));
	memset(&vehicle_local_position_setpoint, 0, sizeof(vehicle_local_position_setpoint));
	memset(&parameter_update, 0, sizeof(parameter_update));
	memset(&param_handles, 0, sizeof(param_handles));
	memset(&params, 0, sizeof(params));

	params.pos_p.zero();
	params.vel_p.zero();
	params.vel_i.zero();
	params.vel_d.zero();
	params.vel_max.zero();
	params.vel_ff.zero();
	params.sp_offs_max.zero();

	param_handles.thr_min = param_find("MPC_THR_MIN");
	param_handles.thr_max = param_find("MPC_THR_MAX");
	param_handles.z_p = param_find("MPC_Z_P");
	param_handles.z_vel_p = param_find("MPC_Z_VEL_P");
	param_handles.z_vel_i = param_find("MPC_Z_VEL_I");
	param_handles.z_vel_d = param_find("MPC_Z_VEL_D");
	param_handles.z_vel_max = param_find("MPC_Z_VEL_MAX");
	param_handles.z_ff = param_find("MPC_Z_FF");
	param_handles.xy_p = param_find("MPC_XY_P");
	param_handles.xy_vel_p = param_find("MPC_XY_VEL_P");
	param_handles.xy_vel_i = param_find("MPC_XY_VEL_I");
	param_handles.xy_vel_d = param_find("MPC_XY_VEL_D");
	param_handles.xy_vel_max = param_find("MPC_XY_VEL_MAX");
	param_handles.xy_ff = param_find("MPC_XY_FF");
	param_handles.tilt_max_air = param_find("MPC_TILTMAX_AIR");
	param_handles.land_speed = param_find("MPC_LAND_SPEED");
	param_handles.tilt_max_land = param_find("MPC_TILTMAX_LND");
	updateParams(true);
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
	orb_unsubscribe(vehicle_local_position_subscription);
	orb_unsubscribe(parameter_update_subscription);
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
	if (needsUpdate(vehicle_local_position_subscription)) {
		orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_subscription, &vehicle_local_position);
	}
}

void UOrbBridge::publish() {
	if (vehicle_attitude_setpoint_publication > 0) {
		orb_publish(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_publication, &vehicle_attitude_setpoint);
	}
	else {
		vehicle_attitude_setpoint_publication = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &vehicle_attitude_setpoint);
	}
	if (vehicle_local_position_setpoint_publication > 0) {
		orb_publish(ORB_ID(vehicle_local_position_setpoint), vehicle_local_position_setpoint_publication, &vehicle_local_position_setpoint);
	}
	else {
		vehicle_local_position_setpoint_publication = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &vehicle_local_position_setpoint);
	}
}

MulticopterPositionControlD3::MulticopterPositionControlD3() :
		_control_task(-1),
		_task_should_exit(false),
		_mavlink_fd(-1),
		uorb(nullptr) {
	memset(&state, 0, sizeof(state));
	state.pos.zero();
	state.pos_sp.zero();
	state.vel.zero();
	state.vel_sp.zero();
	state.vel_prev.zero();
	state.vel_ff.zero();
	state.sp_move_rate.zero();
	state.ref_alt = 0.0f;
	state.ref_timestamp = 0.0f;
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

float MulticopterPositionControlD3::scale_control(float ctl, float end, float dz) {
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	}
	else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	}
	else {
		return 0.0f;
	}
}

void MulticopterPositionControlD3::update_ref() {
	vehicle_local_position_s vehicleLocalPosition = uorb->vehicle_local_position;
	if (vehicleLocalPosition.ref_timestamp != state.ref_timestamp) {
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (state.ref_timestamp != 0) {
			/* calculate current position setpoint in global frame */
			map_projection_reproject(&state.ref_pos, state.pos_sp(0), state.pos_sp(1), &lat_sp, &lon_sp);
			alt_sp = state.ref_alt - state.pos_sp(2);
		}

		/* update local projection reference */
		map_projection_init(&state.ref_pos, vehicleLocalPosition.ref_lat, vehicleLocalPosition.ref_lon);
		state.ref_alt = vehicleLocalPosition.ref_alt;

		if (state.ref_timestamp != 0) {
			/* reproject position setpoint to new reference */
			map_projection_project(&state.ref_pos, lat_sp, lon_sp, &state.pos_sp.data[0], &state.pos_sp.data[1]);
			state.pos_sp(2) = -(alt_sp - state.ref_alt);
		}
		state.ref_timestamp = vehicleLocalPosition.ref_timestamp;
	}
}

void MulticopterPositionControlD3::reset_pos_sp() {
	//if (_reset_pos_sp) {
	//	_reset_pos_sp = false;
	/* shift position setpoint to make attitude setpoint continuous */
	vehicle_attitude_setpoint_s att_sp = uorb->vehicle_attitude_setpoint;
	state.pos_sp(0) = state.pos(0) + (state.vel(0) - att_sp.R_body[0][2] * att_sp.thrust / uorb->params.vel_p(0) - uorb->params.vel_ff(0) * state.sp_move_rate(0)) / uorb->params.pos_p(0);
	state.pos_sp(1) = state.pos(1) + (state.vel(1) - att_sp.R_body[1][2] * att_sp.thrust / uorb->params.vel_p(1) - uorb->params.vel_ff(1) * state.sp_move_rate(1)) / uorb->params.pos_p(1);
	mavlink_log_info(_mavlink_fd, "[mpc] reset pos sp: %d, %d", (int )state.pos_sp(0), (int )state.pos_sp(1));
//	}
}

void MulticopterPositionControlD3::reset_alt_sp() {
	//if (_reset_alt_sp) {
	//	_reset_alt_sp = false;
	state.pos_sp(2) = state.pos(2) + (state.vel(2) - uorb->params.vel_ff(2) * state.sp_move_rate(2)) / uorb->params.pos_p(2);
	mavlink_log_info(_mavlink_fd, "[mpc] reset alt sp: %d", -(int )state.pos_sp(2));
	//}
}

void MulticopterPositionControlD3::limit_pos_sp_offset() {
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();

	if (uorb->vehicle_control_mode.flag_control_position_enabled) {
		pos_sp_offs(0) = (state.pos_sp(0) - state.pos(0)) / uorb->params.sp_offs_max(0);
		pos_sp_offs(1) = (state.pos_sp(1) - state.pos(1)) / uorb->params.sp_offs_max(1);
	}

	if (uorb->vehicle_control_mode.flag_control_altitude_enabled) {
		pos_sp_offs(2) = (state.pos_sp(2) - state.pos(2)) / uorb->params.sp_offs_max(2);
	}

	float pos_sp_offs_norm = pos_sp_offs.length();

	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		state.pos_sp = state.pos + pos_sp_offs.emult(uorb->params.sp_offs_max);
	}
}

void MulticopterPositionControlD3::control_manual(float dt) {
	state.sp_move_rate.zero();

	if (uorb->vehicle_control_mode.flag_control_altitude_enabled) {
		/* move altitude setpoint with throttle stick */
		state.sp_move_rate(2) = -scale_control(state.manualZ - 0.5f, 0.5f, alt_ctl_dz);
	}

	if (uorb->vehicle_control_mode.flag_control_position_enabled) {
		/* move position setpoint with roll/pitch stick */
		state.sp_move_rate(0) = state.manualX;
		state.sp_move_rate(1) = state.manualY;

		/*bool newTargetDetection = lastTargetTimestampExternal != _d3_target.timestamp;
		 lastTargetTimestampExternal = _d3_target.timestamp;
		 if (newTargetDetection) {
		 lastTargetTimestamp = hrt_absolute_time();
		 }

		 bool manualXYinput = !isnan(_manual.x) && !isnan(_manual.y) && (fabsf(_manual.x) > 0.01f || fabsf(_manual.y) > 0.01f);
		 if (!manualXYinput) {
		 hrt_abstime targetAge = hrt_absolute_time() - lastTargetTimestamp;
		 //only accept target detection younger than 0.5 sec
		 if (targetAge < 500000) {
		 float q = ((500000.0f - targetAge) / 500000.0f) * 0.3f; //0-1.0
		 float targetRadX = _d3_target.x;
		 float targetRadY = _d3_target.y;
		 state.sp_move_rate(0) = targetRadY * q;
		 state.sp_move_rate(0) = -targetRadX * q;
		 }
		 }*/
	}

	/* limit setpoint move rate */
	float sp_move_norm = state.sp_move_rate.length();

	if (sp_move_norm > 1.0f) {
		state.sp_move_rate /= sp_move_norm;
	}

	/* state.sp_move_rate scaled to 0..1, scale it to max speed and rotate around yaw */
	math::Matrix<3, 3> R_yaw_sp;
	R_yaw_sp.from_euler(0.0f, 0.0f, uorb->vehicle_attitude_setpoint.yaw_body);
	state.sp_move_rate = R_yaw_sp * state.sp_move_rate.emult(uorb->params.vel_max);

	if (uorb->vehicle_control_mode.flag_control_altitude_enabled) {
		/* reset alt setpoint to current altitude if needed */
		reset_alt_sp();
	}

	if (uorb->vehicle_control_mode.flag_control_position_enabled) {
		/* reset position setpoint to current position if needed */
		reset_pos_sp();
	}

	/* feed forward setpoint move rate with weight vel_ff */
	state.vel_ff = state.sp_move_rate.emult(uorb->params.vel_ff);

	/* move position setpoint */
	state.pos_sp += state.sp_move_rate * dt;

	/* check if position setpoint is too far from actual position */
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();

	if (uorb->vehicle_control_mode.flag_control_position_enabled) {
		pos_sp_offs(0) = (state.pos_sp(0) - state.pos(0)) / uorb->params.sp_offs_max(0);
		pos_sp_offs(1) = (state.pos_sp(1) - state.pos(1)) / uorb->params.sp_offs_max(1);
	}

	if (uorb->vehicle_control_mode.flag_control_altitude_enabled) {
		pos_sp_offs(2) = (state.pos_sp(2) - state.pos(2)) / uorb->params.sp_offs_max(2);
	}

	float pos_sp_offs_norm = pos_sp_offs.length();

	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		state.pos_sp = state.pos + pos_sp_offs.emult(uorb->params.sp_offs_max);
	}
}

void MulticopterPositionControlD3::resetSetpointsIfNeeded() {
	if ((uorb->vehicle_control_mode.flag_armed && !state.armed) || (checkEnablement() && !state.enabled)) {
		state.rollSetpoint = 0.0f;
		state.pitchSetpoint = 0.0f;
		state.yawSetpoint = uorb->vehicle_attitude.yaw;
		state.groundDistLocalSP = state.groundDistLocal;
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
		if (fabsf(rcZ) > 0.2f) {
			state.groundDistLocalSP += rcZ * dt;
		}
		state.yawSetpoint = _wrap_pi(state.yawSetpoint + state.manualR * dt);
		state.rollSetpoint = state.manualY * 0.6f;
		state.pitchSetpoint = -state.manualX * 0.6f;
		float diff = state.groundDistLocalSP - state.groundDistLocal;
		state.thrustSetpoint += min(diff * 0.1f, 0.1f);
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

