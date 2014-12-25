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
		updated(false),
		newTarget(false) {
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
		newTarget = true;
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

void UOrbBridge::publishAttitudeSetpoint() {
	if (vehicle_attitude_setpoint_publication > 0) {
		orb_publish(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_publication, &vehicle_attitude_setpoint);
	}
	else {
		vehicle_attitude_setpoint_publication = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &vehicle_attitude_setpoint);
	}
}

void UOrbBridge::publishLocalPositionSetpoint() {
	if (vehicle_local_position_setpoint_publication > 0) {
		orb_publish(ORB_ID(vehicle_local_position_setpoint), vehicle_local_position_setpoint_publication, &vehicle_local_position_setpoint);
	}
	else {
		vehicle_local_position_setpoint_publication = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &vehicle_local_position_setpoint);
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
	/* shift position setpoint to make attitude setpoint continuous */
	state.pos_sp(0) = state.pos(0);
	state.pos_sp(1) = state.pos(1);
	state.pos_err.zero();
	mavlink_log_info(_mavlink_fd, "[mpc] reset pos sp: %d, %d", (int )state.pos_sp(0), (int )state.pos_sp(1));
}

void MulticopterPositionControlD3::reset_alt_sp() {
	state.pos_sp(2) = state.pos(2);
	mavlink_log_info(_mavlink_fd, "[mpc] reset alt sp: %d", (int )state.pos_sp(2));
}

void MulticopterPositionControlD3::control_manual(float dt) {
	float altPosChange = 0.0f;
	if (uorb->vehicle_control_mode.flag_control_altitude_enabled) {
		/* move altitude setpoint with throttle stick */
		altPosChange = -scale_control(state.manualZ - 0.5f, 0.5f, alt_ctl_dz);
		if (fabsf(altPosChange) >= 0.1f) {
			state.integral_z_frozen = true;
		}
		state.pos_sp(2) += altPosChange * dt;
	}
	if (uorb->vehicle_control_mode.flag_control_position_enabled) {
		/* move position setpoint with roll/pitch stick */
		vehicle_attitude_s vehicleAttitude = uorb->vehicle_attitude;
		float manualX = state.manualX;
		float manualY = state.manualY;
		float frame_m[3];
		frame_m[0] = manualX;
		frame_m[1] = manualY;
		frame_m[2] = 0.0f;
		float manualChangeNED[2] = { 0.0f, 0.0f };
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 3; j++) {
				manualChangeNED[i] += vehicleAttitude.R[i][j] * frame_m[j];
			}
		}
		state.pos_sp(0) += manualChangeNED[0] * dt * 10.0f;
		state.pos_sp(1) += manualChangeNED[1] * dt * 10.0f;
	}
}

void MulticopterPositionControlD3::resetSetpointsIfNeeded(float dt) {
	if ((uorb->vehicle_control_mode.flag_armed && !state.armed) || (checkEnablement() && !state.enabled)) {
		reset_pos_sp();
		reset_alt_sp();
		state.thrust_int.zero();
		state.thrust_int(2) = -state.manualZ;
		state.vel_err.zero();
		calculateThrustSetpointWithPID(dt);
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
	state.manualXYinput = fabsf(state.manualX) > 0.1f || fabsf(state.manualY) > 0.1f;
	if (state.manualXYinput) {
		state.integral_xy_frozen = true;
	}
	if (uorb->vehicle_control_mode.flag_control_manual_enabled) {
		control_manual(dt);
	}
}

void MulticopterPositionControlD3::applyTargetInput(hrt_abstime currrentTimestamp) {
	if (!state.manualXYinput && uorb->newTarget) {
		uorb->newTarget = false;
		state.integral_xy_frozen = true;
		d3_target_s d3Target = uorb->d3_target;
		vehicle_attitude_s vehicleAttitude = uorb->vehicle_attitude;
		vehicle_local_position_s vehicleLocalPosition = uorb->vehicle_local_position;
		float distBottom = vehicleLocalPosition.dist_bottom;

		float targetRadX = -d3Target.y;
		float targetRadY = -d3Target.x;
		float frame_m[3];
		frame_m[0] = targetRadX * distBottom;
		frame_m[1] = targetRadY * distBottom;
		frame_m[2] = 0.0f;

		float targetPosNED[2] = { 0.0f, 0.0f };
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 3; j++) {
				targetPosNED[i] += vehicleAttitude.R[i][j] * frame_m[j];
			}
		}
		state.pos_sp(0) = state.pos(0) + targetPosNED[0];
		state.pos_sp(1) = state.pos(1) + targetPosNED[1];
	}
}

void MulticopterPositionControlD3::calculateAttitudeSP() {
	/* calculate attitude setpoint from thrust vector */
	/* desired body_z axis = -normalize(thrust_vector) */
	math::Vector<3> body_x;
	math::Vector<3> body_y;
	math::Vector<3> body_z;
	if (state.thrust_abs > SIGMA) {
		body_z = -state.thrust_sp / state.thrust_abs;
	}
	else {
		/* no thrust, set Z axis to safe value */
		body_z.zero();
		body_z(2) = 1.0f;
	}
	/* vector of desired yaw direction in XY plane, rotated by PI/2 */
	math::Vector<3> y_C(-sinf(uorb->vehicle_attitude_setpoint.yaw_body), cosf(uorb->vehicle_attitude_setpoint.yaw_body), 0.0f);
	if (fabsf(body_z(2)) > SIGMA) {
		/* desired body_x axis, orthogonal to body_z */
		body_x = y_C % body_z;

		/* keep nose to front while inverted upside down */
		if (body_z(2) < 0.0f) {
			body_x = -body_x;
		}
		body_x.normalize();
	}
	else {
		/* desired thrust is in XY plane, set X downside to construct correct matrix,
		 * but yaw component will not be used actually */
		body_x.zero();
		body_x(2) = 1.0f;
	}
	/* desired body_y axis */
	body_y = body_z % body_x;
	/* fill rotation matrix */
	for (int i = 0; i < 3; i++) {
		state.R(i, 0) = body_x(i);
		state.R(i, 1) = body_y(i);
		state.R(i, 2) = body_z(i);
	}
}

void MulticopterPositionControlD3::publishAttitudeSP() {
	/* copy rotation matrix to attitude setpoint topic */
	memcpy(&uorb->vehicle_attitude_setpoint.R_body[0][0], state.R.data, sizeof(uorb->vehicle_attitude_setpoint.R_body));
	uorb->vehicle_attitude_setpoint.R_valid = true;
	/* calculate euler angles, for logging only, must not be used for control */
	math::Vector<3> euler = state.R.to_euler();
	uorb->vehicle_attitude_setpoint.roll_body = euler(0);
	uorb->vehicle_attitude_setpoint.pitch_body = euler(1);
	/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */
	uorb->vehicle_attitude_setpoint.thrust = state.thrust_abs;
	uorb->vehicle_attitude_setpoint.timestamp = hrt_absolute_time();
	uorb->publishAttitudeSetpoint();
}

void MulticopterPositionControlD3::updateIntegrals(float dt) {
	/* update integrals */
	if (!state.integral_xy_frozen) {
		state.thrust_int(0) += state.pos_err(0) * uorb->params.vel_i(0) * dt;
		state.thrust_int(1) += state.pos_err(1) * uorb->params.vel_i(1) * dt;
	}
	if (!state.integral_z_frozen) {
		state.thrust_int(2) += state.pos_err(2) * uorb->params.vel_i(2) * dt;
		/* protection against flipping on ground when landing */
		if (state.thrust_int(2) > 0.0f) {
			state.thrust_int(2) = 0.0f;
		}
	}
}

void MulticopterPositionControlD3::limitMaxThrust() {
	/* limit thrust vector and check for saturation */
	/* limit min lift */
	float thr_min = uorb->params.thr_min;
	float tilt_max = uorb->params.tilt_max_air;
	/* limit min lift */
	if (-state.thrust_sp(2) < thr_min) {
		state.thrust_sp(2) = -thr_min;
		state.integral_z_frozen = true;
	}

	/* limit max tilt */
	/* absolute horizontal thrust */
	float thrust_sp_xy_len = math::Vector<2>(state.thrust_sp(0), state.thrust_sp(1)).length();
	if (thrust_sp_xy_len > 0.01f) {
		/* max horizontal thrust for given vertical thrust*/
		float thrust_xy_max = -state.thrust_sp(2) * tanf(tilt_max);

		if (thrust_sp_xy_len > thrust_xy_max) {
			float k = thrust_xy_max / thrust_sp_xy_len;
			state.thrust_sp(0) *= k;
			state.thrust_sp(1) *= k;
			state.integral_xy_frozen = true;
		}
	}
	/* limit max thrust */
	state.thrust_abs = state.thrust_sp.length();
	if (state.thrust_abs > uorb->params.thr_max) {
		if (state.thrust_sp(2) < 0.0f) {
			if (-state.thrust_sp(2) > uorb->params.thr_max) {
				/* thrust Z component is too large, limit it */
				state.thrust_sp(0) = 0.0f;
				state.thrust_sp(1) = 0.0f;
				state.thrust_sp(2) = -uorb->params.thr_max;
				state.integral_xy_frozen = true;
				state.integral_z_frozen = true;

			}
			else {
				/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
				float thrust_xy_max = sqrtf(uorb->params.thr_max * uorb->params.thr_max - state.thrust_sp(2) * state.thrust_sp(2));
				float thrust_xy_abs = math::Vector<2>(state.thrust_sp(0), state.thrust_sp(1)).length();
				float k = thrust_xy_max / thrust_xy_abs;
				state.thrust_sp(0) *= k;
				state.thrust_sp(1) *= k;
				state.integral_xy_frozen = true;
			}

		}
		else {
			/* Z component is negative, going down, simply limit thrust vector */
			float k = uorb->params.thr_max / state.thrust_abs;
			state.thrust_sp *= k;
			state.integral_xy_frozen = true;
			state.integral_z_frozen = true;
		}

		state.thrust_abs = uorb->params.thr_max;
	}
}

void MulticopterPositionControlD3::getLocalPos() {
	vehicle_local_position_s localPos = uorb->vehicle_local_position;
	state.pos(0) = localPos.x;
	state.pos(1) = localPos.y;
	state.pos(2) = localPos.z;
	state.vel(0) = localPos.vx;
	state.vel(1) = localPos.vy;
	state.vel(2) = localPos.vz;
}

void MulticopterPositionControlD3::fillAndPubishLocalPositionSP() {
	uorb->vehicle_local_position_setpoint.timestamp = hrt_absolute_time();
	uorb->vehicle_local_position_setpoint.x = state.pos_sp(0);
	uorb->vehicle_local_position_setpoint.y = state.pos_sp(1);
	uorb->vehicle_local_position_setpoint.z = state.pos_sp(2);
	uorb->vehicle_local_position_setpoint.yaw = uorb->vehicle_attitude_setpoint.yaw_body;
	uorb->publishLocalPositionSetpoint();
}

void MulticopterPositionControlD3::calculateThrustSetpointWithPID(float dt) {
	Vector<3> posP = uorb->params.pos_p;
	Vector<3> velP = uorb->params.vel_p;
	Vector<3> velD = uorb->params.vel_d;
	Vector<3> oldPosErr = state.pos_err;
	state.pos_err = state.pos_sp - state.pos;
	Vector<3> posErrorDelta = (state.pos_err - oldPosErr) / dt;
	state.vel_sp = state.pos_err.emult(posP);
	Vector<3> velErrNew = state.vel_sp - state.vel;
	state.vel_err_d = (velErrNew - state.vel_err) / dt;
	state.vel_err = velErrNew;
	state.thrust_sp = state.vel_err.emult(velP) + state.vel_err_d.emult(velD) + posErrorDelta.emult(velD) + state.thrust_int;
}

void MulticopterPositionControlD3::doLoop() {
	uorb->update();
	uorb->updateParams(false);
	hrt_abstime currrentTimestamp = hrt_absolute_time();
	float dt = state.lastTimestamp != 0 ? (currrentTimestamp - state.lastTimestamp) * 0.000001f : 0.0f;
	state.lastTimestamp = currrentTimestamp;

	state.integral_xy_frozen = false;
	state.integral_z_frozen = false;

	resetSetpointsIfNeeded(dt);
	update_ref();
	getLocalPos();
	applyRCInputIfAvailable(dt);
	applyTargetInput(currrentTimestamp);
	fillAndPubishLocalPositionSP();

	if (state.enabled) {
		calculateThrustSetpointWithPID(dt);
		limitMaxThrust();
		updateIntegrals(dt);
		calculateAttitudeSP();
		publishAttitudeSP();
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

