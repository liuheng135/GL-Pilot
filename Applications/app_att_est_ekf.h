#ifndef __APP_ATT_EST_EKH_H_
#define __APP_ATT_EST_EKH_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <float.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB.h>
#include <drivers.h>

#include "app_sensors.h"

#include "attitudeKalmanfilter_initialize.h"
#include "attitudeKalmanfilter.h"


typedef struct{

	uint64_t timestamp;	/**< in microseconds since system start          */

	/* This is similar to the mavlink message ATTITUDE, but for onboard use */

	/** @warning roll, pitch and yaw have always to be valid, the rotation matrix and quaternion are optional */

	float roll;		/**< Roll angle (rad, Tait-Bryan, NED)				*/
	float pitch;		/**< Pitch angle (rad, Tait-Bryan, NED)				*/
	float yaw;		/**< Yaw angle (rad, Tait-Bryan, NED)				*/
	float rollspeed;	/**< Roll angular speed (rad/s, Tait-Bryan, NED)		*/
	float pitchspeed;	/**< Pitch angular speed (rad/s, Tait-Bryan, NED)		*/
	float yawspeed;		/**< Yaw angular speed (rad/s, Tait-Bryan, NED)			*/
	float rollacc;		/**< Roll angular accelration (rad/s, Tait-Bryan, NED)		*/
	float pitchacc;		/**< Pitch angular acceleration (rad/s, Tait-Bryan, NED)	*/
	float yawacc;		/**< Yaw angular acceleration (rad/s, Tait-Bryan, NED)		*/
	float rate_offsets[3];	/**< Offsets of the body angular rates from zero		*/
	float R[3][3];		/**< Rotation matrix body to world, (Tait-Bryan, NED)		*/
	float q[4];		/**< Quaternion (NED)						*/
	float g_comp[3];	/**< Compensated gravity vector					*/
	bool R_valid;		/**< Rotation matrix valid					*/
	bool q_valid;		/**< Quaternion valid						*/

}vehicle_attitude_s;

typedef struct {
	float r[9];
	float q[12];
	float roll_off;
	float pitch_off;
	float yaw_off;
}attestimatorekfParams;


extern void app_att_est_ekf_main(void* parameter);

#endif
