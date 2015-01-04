#include "app_att_est_ekf.h"



static bool att_est_ekf_should_exit = false;		/**< Deamon exit flag */
static bool att_est_ekf_running = false;		/**< Deamon status flag */

attestimatorekfParams  ekf_params;

ORB_DEFINE(vehicle_attitude,vehicle_attitude_s);

void euler_to_rot_mat(float r[3][3],float roll, float pitch, float yaw) 
{
		float cp = cosf(pitch);
		float sp = sinf(pitch);
		float sr = sinf(roll);
		float cr = cosf(roll);
		float sy = sinf(yaw);
		float cy = cosf(yaw);

		r[0][0] = cp * cy;
		r[0][1] = (sr * sp * cy) - (cr * sy);
		r[0][2] = (cr * sp * cy) + (sr * sy);
		r[1][0] = cp * sy;
		r[1][1] = (sr * sp * sy) + (cr * cy);
		r[1][2] = (cr * sp * sy) - (sr * cy);
		r[2][0] = -sp;
		r[2][1] = sr * cp;
		r[2][2] = cr * cp;
}

/*
 * [Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
 */

/*
 * EKF Attitude Estimator main function.
 *
 * Estimates the attitude recursively once started.
 *
 * @param argc number of commandline arguments (plus command name)
 * @param argv strings containing the arguments
 */
void app_att_est_ekf_main(void* parameter)
{

const unsigned int loop_interval_alarm = 6500;	// loop interval in microseconds

	float dt = 0.005f;
/* state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]' */
	float z_k[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f, 0.2f, -0.2f, 0.2f};					/**< Measurement vector */
	float x_aposteriori_k[12];		/**< states */
	float P_aposteriori_k[144] = {100.f, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
				     0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
				     0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,
				     0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,
				     0,   0,   0,   0,  100.f,  0,   0,   0,   0,   0,   0,   0,
				     0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,
				     0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,
				     0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,
				     0,   0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,
				     0,   0,   0,   0,   0,   0,   0,   0,  0.0f, 100.0f,   0,   0,
				     0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   100.0f,   0,
				     0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   0,   100.0f,
				    }; /**< init: diagonal matrix with big values */

	float x_aposteriori[12];
	float P_aposteriori[144];

	/* output euler angles */
	float euler[3] = {0.0f, 0.0f, 0.0f};

	float Rot_matrix[9] = {1.f,  0,  0,
			      0,  1.f,  0,
			      0,  0,  1.f
			     };		/**< init: identity matrix */

	// print text
	printf("Extended Kalman Filter Attitude Estimator initialized..\n\n");
	fflush(stdout);

	int overloadcounter = 19;

	/* Initialize filter */
	attitudeKalmanfilter_initialize();

	/* store start time to guard against too slow update rates */
	uint64_t last_run = hrt_absolute_time();

	sensor_combined_s raw;
	rt_memset(&raw, 0, sizeof(raw));
	//struct vehicle_gps_position_s gps;
	//rt_memset(&gps, 0, sizeof(gps));
	//struct vehicle_global_position_s global_pos;
	//rt_memset(&global_pos, 0, sizeof(global_pos));
	vehicle_attitude_s att;
	rt_memset(&att, 0, sizeof(att));
	//struct vehicle_control_mode_s control_mode;
	//rt_memset(&control_mode, 0, sizeof(control_mode));

	uint64_t last_data = 0;
	uint64_t last_measurement = 0;
    rt_uint32_t sensors_sub = 0;
	uint64_t last_timestamp_acc = 0;
	uint64_t last_timestamp_gyro = 0;
	uint64_t last_timestamp_mag = 0;

	/* rotation matrix */
	float R[3][3] = {1,0,0,
					 0,1,0,
					 0,0,1};

	/* subscribe to raw data */
	orb_subscribe(ORB_ID(sensor_combined),&sensors_sub);

	orb_advertise(ORB_ID(vehicle_attitude), &att);

	att_est_ekf_running = true;

	bool initialized = false;

	/* magnetic declination, in radians */
	float mag_decl = 0.0f;

	/* rotation matrix for magnetic declination */
	float R_mag[3][3] = {1,0,0,
						 0,1,0,
						 0,0,1};

    uint8_t update_vect[3] = {0, 0, 0};
	
	rt_memset(&ekf_params, 0, sizeof(ekf_params));
	ekf_params.q[0] = 1e-4f;
	ekf_params.q[1] = 0.08f;
	ekf_params.q[2] = 0.009f;
	ekf_params.q[3] = 0.005f;
	ekf_params.q[4] = 0.0f;
	
	ekf_params.r[0] = 0.0008f;
	ekf_params.r[1] = 10000.0f;
	ekf_params.r[2] = 100.0f;
	ekf_params.r[3] = 0.0f;
	
	/* Main loop*/
	while (!att_est_ekf_should_exit) {   
		/* only run filter if sensor values changed */
		if (orb_check(&sensors_sub,5000) == RT_EOK) {

			/* get latest measurements */
			orb_copy(ORB_ID(sensor_combined), &raw);
		}
		else{
			rt_kprintf("sensors data lost!\n");
		}
		if (!initialized) {
			initialized = true;
		}
		else {
			/* Calculate data time difference in seconds */
			dt = (raw.acc_timestamp - last_measurement) / 1000000.0f;
			last_measurement = raw.acc_timestamp;
			
			if(raw.gyro_timestamp != last_timestamp_gyro)
			{
				update_vect[0] = 1;
				last_timestamp_gyro = raw.gyro_timestamp;
			}
			if(raw.acc_timestamp != last_timestamp_acc)
			{
				update_vect[1] = 1;
				last_timestamp_acc = raw.acc_timestamp;
			}
			if(raw.mag_timestamp != last_timestamp_mag)
			{
				update_vect[2] = 1;
				last_timestamp_mag = raw.mag_timestamp;
			}
			
			z_k[0] =  raw.gyro_rad_s[0];
			z_k[1] =  raw.gyro_rad_s[1];
			z_k[2] =  raw.gyro_rad_s[2];

			z_k[3] = raw.acc_m_s2[0];
			z_k[4] = raw.acc_m_s2[1];
			z_k[5] = raw.acc_m_s2[2];

			z_k[6] = raw.mag_ga[0];
			z_k[7] = raw.mag_ga[1];
			z_k[8] = raw.mag_ga[2];

			uint64_t now = hrt_absolute_time();
			unsigned int time_elapsed = now - last_run;
			last_run = now;

			if (time_elapsed > loop_interval_alarm) {

				/* cpu overload */

				overloadcounter++;
			}

			static bool const_initialized = false;

			/* initialize with good values once we have a reasonable dt estimate */
			if (!const_initialized && dt < 0.05f && dt > 0.001f) {
				dt = 0.005f;
				
				/* update mag declination rotation matrix */
				euler_to_rot_mat(R_mag,0.0f, 0.0f, mag_decl);

				x_aposteriori_k[0] = z_k[0];
				x_aposteriori_k[1] = z_k[1];
				x_aposteriori_k[2] = z_k[2];
				x_aposteriori_k[3] = 0.0f;
				x_aposteriori_k[4] = 0.0f;
				x_aposteriori_k[5] = 0.0f;
				x_aposteriori_k[6] = z_k[3];
				x_aposteriori_k[7] = z_k[4];
				x_aposteriori_k[8] = z_k[5];
				x_aposteriori_k[9] = z_k[6];
				x_aposteriori_k[10] = z_k[7];
				x_aposteriori_k[11] = z_k[8];

				const_initialized = true;
			}

			/* do not execute the filter if not initialized */
			if (!const_initialized) {
				continue;
			}

			attitudeKalmanfilter(update_vect, dt, z_k, x_aposteriori_k, P_aposteriori_k, ekf_params.q, ekf_params.r,
						 euler, Rot_matrix, x_aposteriori, P_aposteriori);

			
			for(int i = 0;i < 3;i++)
			{
				update_vect[i] = 0;
			}
			/* swap values for next iteration, check for fatal inputs */
			if (isfinite(euler[0]) && isfinite(euler[1]) && isfinite(euler[2])) {
				rt_memcpy(P_aposteriori_k, P_aposteriori, sizeof(P_aposteriori_k));
				rt_memcpy(x_aposteriori_k, x_aposteriori, sizeof(x_aposteriori_k));

			} else {
				/* due to inputs or numerical failure the output is invalid, skip it */
				continue;
			}

			if (last_data > 0 && raw.acc_timestamp - last_data > 30000)
				printf("[attitude estimator ekf] sensor data missed! (%llu)\n", raw.acc_timestamp - last_data);

			last_data = raw.acc_timestamp;

			/* send out */
			att.timestamp = raw.acc_timestamp;

			att.roll = euler[0];
			att.pitch = euler[1];
			att.yaw = euler[2] + mag_decl;

			att.rollspeed = x_aposteriori[0];
			att.pitchspeed = x_aposteriori[1];
			att.yawspeed = x_aposteriori[2];
			att.rollacc = x_aposteriori[3];
			att.pitchacc = x_aposteriori[4];
			att.yawacc = x_aposteriori[5];

			att.g_comp[0] = raw.acc_m_s2[0];
			att.g_comp[1] = raw.acc_m_s2[1];
			att.g_comp[2] = raw.acc_m_s2[2];

			/* copy offsets */
			rt_memcpy(&att.rate_offsets, &(x_aposteriori[3]), sizeof(att.rate_offsets));

			/* magnetic declination */
			float R_body[3][3] = {0};
			rt_memcpy(&R_body, &Rot_matrix, sizeof(R_body));
			
			//R = R_mag * R_body;
			for(int i = 0;i < 3;i++){
				for(int j = 0;j < 3;i++)
				{
					R[i][j] = 0;
					for(int k = 0;k < 3;k++)
						R[i][j] += R_mag[i][k] * R_body[k][j];
				}
			}
						
			
			/* copy rotation matrix */
			rt_memcpy(&att.R, &R, sizeof(att.R));
			att.R_valid = true;

			if (isfinite(att.roll) && isfinite(att.pitch) && isfinite(att.yaw)) {
				// Broadcast
				orb_publish(ORB_ID(vehicle_attitude), &att);

			} else {
				rt_kprintf("NaN in roll/pitch/yaw estimate!");
			}
		}
	}
	att_est_ekf_running = false;
}
