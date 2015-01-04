#ifndef __APP_SENSORS_H_
#define __APP_SENSORS_H_

#include "rtthread.h"
#include "uorb.h"
#include "drivers.h"

#define IMU_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000
#define IMU_G_PER_LSB_CFG     MPU6500_G_PER_LSB_8
#define IMU_1G_RAW            (int16_t)(1.0f / IMU_G_PER_LSB_CFG)

#define G                     9.8143f



typedef struct
{
	int16_t    acc_raw[3];
	float      acc_m_s2[3];
	
	int16_t    gyro_raw[3];
	float      gyro_rad_s[3];
	
	int16_t    mag_raw[3];
	float      mag_ga[3];
	
	float      baro_pres_mbar;			/**< Barometric pressure, already temp. comp.     */
	float      baro_alt_meter;			/**< Altitude, already temp. comp.                */
	float      baro_temp_celcius;	
	
	uint64_t   acc_timestamp;
	uint64_t   mag_timestamp;
	uint64_t   gyro_timestamp;
}sensor_combined_s;

ORB_DECLARE(sensor_combined);

void app_sensors_main(void* parameter);

#endif
