/*
 * mc_pos_control_d3.h
 *
 *  Created on: Dec 5, 2014
 *      Author: helios
 */

#ifndef MC_POS_CONTROL_D3_H_
#define MC_POS_CONTROL_D3_H_


#include <stdbool.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <systemlib/param/param.h>
#include <uORB/uORB.h>

namespace pos_control_d3 {

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f

class MulticopterPositionControlD3 {
	public:
		MulticopterPositionControlD3();
		~MulticopterPositionControlD3();
		void main_loop();
	private:
		bool _task_should_exit; /**< if true, task should exit */
		int _control_task; /**< task handle for task */
		int _mavlink_fd; /**< mavlink fd */

		int _att_sub; /**< vehicle attitude subscription */
		int _att_sp_sub; /**< vehicle attitude setpoint */
		int _control_mode_sub; /**< vehicle control mode subscription */
		int _params_sub; /**< notification of parameter updates */
		int _manual_sub; /**< notification of manual control updates */
		int _arming_sub; /**< arming status of outputs */

		orb_advert_t _att_sp_pub; /**< attitude setpoint publication */

		struct vehicle_attitude_s _att; /**< vehicle attitude */
		struct vehicle_attitude_setpoint_s _att_sp; /**< vehicle attitude setpoint */
		struct manual_control_setpoint_s _manual; /**< r/c channel data */
		struct vehicle_control_mode_s _control_mode; /**< vehicle control mode */
		struct actuator_armed_s _arming; /**< actuator arming status */

		struct {
				param_t thr_min;
				param_t thr_max;
				param_t z_p;
				param_t z_vel_p;
				param_t z_vel_i;
				param_t z_vel_d;
				param_t z_vel_max;
				param_t z_ff;
				param_t xy_p;
				param_t xy_vel_p;
				param_t xy_vel_i;
				param_t xy_vel_d;
				param_t xy_vel_max;
				param_t xy_ff;
				param_t tilt_max_air;
				param_t land_speed;
				param_t tilt_max_land;
		} _params_handles; /**< handles for interesting parameters */

		struct {
				float thr_min;
				float thr_max;
				float tilt_max_air;
				float land_speed;
				float tilt_max_land;

				math::Vector<3> pos_p;
				math::Vector<3> vel_p;
				math::Vector<3> vel_i;
				math::Vector<3> vel_d;
				math::Vector<3> vel_ff;
				math::Vector<3> vel_max;
				math::Vector<3> sp_offs_max;
		} _params;

		struct map_projection_reference_s _ref_pos;
		float _ref_alt;

		bool _reset_pos_sp;
		bool _reset_alt_sp;
		bool _mode_auto;

		math::Vector<3> _pos;
		math::Vector<3> _pos_sp;
		math::Vector<3> _vel;
		math::Vector<3> _vel_sp;
		math::Vector<3> _vel_prev; /**< velocity on previous step */
		math::Vector<3> _vel_ff;
		math::Vector<3> _sp_move_rate;

		/**
		 * Update our local parameter cache.
		 */
		int parameters_update(bool force);

		/**
		 * Update control outputs
		 */
		void control_update();

		/**
		 * Check for changes in subscribed topics.
		 */
		void poll_subscriptions();

		static float scale_control(float ctl, float end, float dz);

		/**
		 * Update reference for local position projection
		 */
		void update_ref();
		/**
		 * Reset position setpoint to current position
		 */
		void reset_pos_sp();

		/**
		 * Reset altitude setpoint to current altitude
		 */
		void reset_alt_sp();

		/**
		 * Check if position setpoint is too far from current position and adjust it if needed.
		 */
		void limit_pos_sp_offset();

		/**
		 * Set position setpoint using manual control
		 */
		void control_manual(float dt);

		/**
		 * Set position setpoint using offboard control
		 */
		void control_offboard(float dt);

		bool cross_sphere_line(const math::Vector<3>& sphere_c, float sphere_r, const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3>& res);

		/**
		 * Set position setpoint for AUTO
		 */
		void control_auto(float dt);

		/**
		 * Select between barometric and global (AMSL) altitudes
		 */
		void select_alt(bool global);
};
} /* namespace pos_control_d3 */

#endif /* MC_POS_CONTROL_D3_H_ */
