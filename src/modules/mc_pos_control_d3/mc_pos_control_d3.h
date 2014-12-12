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

using namespace math;

namespace pos_control_d3 {

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f

	class UOrbBridge {
		private:
			orb_advert_t vehicle_attitude_setpoint_publication; //lazy initialized on first publish
			bool updated;
			int vehicle_attitude_subscription;
			int vehicle_attitude_setpoint_subscription;
			int vehicle_control_mode_subscription;
			int manual_control_setpoint_subscription;
			int actuator_armed_subscription;
			int d3_target_subscription;
			int sensor_combined_subscription;
			int vehicle_gps_position_subscription;
			int optical_flow_subscription;bool needsUpdate(int attSub);
		public:
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
			void update();/** updates all subsctions*/
			void publish();/** publishes all outgoin messages*/
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
					float rollSetpoint;
					float pitchSetpoint;
					float yawSetpoint;
					float thrustSetpoint;
					float manualX;
					float manualY;
					float manualZ;
					float manualR;
					float groundDistLocal;
					float groundDistLocalSP;
					float groundDistMSL;
					float groundDistBaroOffset;
					float groundDistMSLOffset;
					uint64_t targetLastTimestampExternal;
					uint64_t targetLastTimestampLocal;
			} state;
			int _control_task; /**< task handle for task */
			bool _task_should_exit; /**< if true, task should exit */
			MulticopterPositionControlD3();
			~MulticopterPositionControlD3();
			void main_loop();
		private:
			int _mavlink_fd; /**< mavlink fd */
			UOrbBridge *uorb;
			void initialize(); //
			void doLoop();
			void publishAttitudeSetpoint();
			void resetSetpointsIfNeeded();bool checkEnablement();
			void applyRCInputIfAvailable(float dt);bool newTargetDetected();
			void applyTargetInput(hrt_abstime currrentTimestamp);
			float filterGroundDist(float groundDistLocalOld, float groundDistSonarM);
			void calculateGroundDistance();
	};
} /* namespace pos_control_d3 */

#endif /* MC_POS_CONTROL_D3_H_ */
