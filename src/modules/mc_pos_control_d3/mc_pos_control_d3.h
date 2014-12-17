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
#include <mathlib/mathlib.h>
#include <poll.h>
#include <lib/geo/geo.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/d3_target.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace math;

namespace pos_control_d3 {

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f

	class UOrbBridge {
		private:
			orb_advert_t vehicle_attitude_setpoint_publication; //lazy initialized on first publish
			orb_advert_t vehicle_local_position_setpoint_publication; //lazy initialized on first publish
			bool updated;
			int vehicle_attitude_subscription;
			int vehicle_attitude_setpoint_subscription;
			int vehicle_control_mode_subscription;
			int manual_control_setpoint_subscription;
			int actuator_armed_subscription;
			int d3_target_subscription;
			int sensor_combined_subscription;
			int vehicle_gps_position_subscription;
			int optical_flow_subscription;
			int vehicle_local_position_subscription;
			int parameter_update_subscription; //
			bool needsUpdate(int attSub);

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
			} param_handles;

		public:
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
			} params;
			UOrbBridge();
			~UOrbBridge();
			struct vehicle_attitude_s vehicle_attitude;
			struct vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
			struct manual_control_setpoint_s manual_control_setpoint; /**< r/c channel data */
			struct vehicle_control_mode_s vehicle_control_mode;
			struct actuator_armed_s actuator_armed; /**< actuator arming status */
			struct d3_target_s d3_target;
			struct sensor_combined_s sensor_combined;
			struct vehicle_gps_position_s vehicle_gps_position;
			struct optical_flow_s optical_flow;
			struct vehicle_local_position_s vehicle_local_position;
			struct vehicle_local_position_setpoint_s vehicle_local_position_setpoint;
			struct parameter_update_s parameter_update;
			void update();/** updates all subsctions*/
			void updateParams(bool force);
			void publishAttitudeSetpoint();
			void publishLocalPositionSetpoint();
			int getVehicleAttitudeSubscription();
	};

	class MulticopterPositionControlD3 {
		public:
			struct state_s {
					struct pollfd fds[1]; /** wakeup source */
					hrt_abstime lastTimestamp; //
					bool armed; //
					bool enabled; //
					bool manualXYinput;
					Matrix<3, 3> R;
					float manualX;
					float manualY;
					float manualZ;
					float manualR;
					hrt_abstime ref_timestamp;
					struct map_projection_reference_s ref_pos;
					float ref_alt;
					math::Vector<3> pos;
					math::Vector<3> pos_sp;
					math::Vector<3> vel;
					math::Vector<3> vel_sp;
					math::Vector<3> thrust_int;
					math::Vector<3> thrust_sp;
					math::Vector<3> pos_err;
					math::Vector<3> vel_err;
					math::Vector<3> vel_err_d;
					float thrust_abs;bool saturation_xy;bool saturation_z;
			} state;
			int _control_task; /**< task handle for task */
			bool _task_should_exit; /**< if true, task should exit */
			MulticopterPositionControlD3();
			~MulticopterPositionControlD3();
			void main_loop();
		private:
			const float alt_ctl_dz = 0.2f;
			int _mavlink_fd; /**< mavlink fd */
			UOrbBridge *uorb;
			void initialize(); //
			void doLoop();
			void resetSetpointsIfNeeded();bool checkEnablement();
			void applyRCInputIfAvailable(float dt);
			void applyTargetInput(hrt_abstime currrentTimestamp);
			float filterGroundDist(float groundDistLocalOld, float groundDistSonarM);
			void calculateGroundDistance();
			float scale_control(float ctl, float end, float dz);
			void update_ref();
			void reset_pos_sp();
			void reset_alt_sp();
			void limit_pos_sp_offset();
			void control_manual(float dt);
			void calculateAttitudeSP();
			void publishAttitudeSP();
			void updateIntegrals(float dt);
			void limitMaxThrust();
			void getLocalPos();
			void fillAndPubishLocalPositionSP();
	};
} /* namespace pos_control_d3 */

#endif /* MC_POS_CONTROL_D3_H_ */
