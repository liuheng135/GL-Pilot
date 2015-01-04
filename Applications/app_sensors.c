#include "app_sensors.h"
#include "kalmanfilter.h"
#include "stdbool.h"

ORB_DEFINE(sensor_combined,sensor_combined_s);
bool app_sensors_should_exit = false;


void app_sensors_main(void* parameter)
{
	uint32_t i;
	int16_t acc_raw[3];
	int16_t gyro_raw[3];
	int16_t mag_raw[3];
	
	int16_t acc_offset[3] = {0};
	int16_t gyro_offset[3] = {0};
	int16_t mag_offset[3] = {0};
	uint16_t mag_div = 10;
	uint16_t mag_counter = 0;
	uint16_t baro_div =5;
	uint16_t baro_counter = 0;
	sensor_combined_s sensors = {0};
    kalmanfilter acc_filter[3],mag_filter[3];
	

	for(i = 0;i < 3;i++)
	{
		KalmanFilter_Init(&acc_filter[i],10,400);
		KalmanFilter_Init(&mag_filter[i],20,100);
	}
	
	if(!(MPU9250_Init() == RT_EOK))
	{
		while(1)
		{
			rt_thread_delay(RT_TICK_PER_SECOND);
			rt_kprintf("ERROR:MPU9250 is not found !\n");
		}
	}
	
	orb_advertise(ORB_ID(sensor_combined),&sensors);
	
	/*we should get offset of sensors here, but calibration is not work yet. */
	
	/* main loop*/
	while(!app_sensors_should_exit)
	{
		MPU9250_GetMotion9(&acc_raw[0],
		                   &acc_raw[1],
		                   &acc_raw[2],
		                   &gyro_raw[0],
		                   &gyro_raw[1],
		                   &gyro_raw[2],
		                   &mag_raw[0],
		                   &mag_raw[1],
		                   &mag_raw[2]);
		
		/* we can do rotation here if we need. */
		
		for(i = 0;i < 3;i++)
		{
			sensors.acc_raw[i]  = acc_raw[i];
			sensors.acc_m_s2[i]   = (acc_raw[i] - acc_offset[i]) * IMU_G_PER_LSB_CFG * G;
			sensors.acc_timestamp = hrt_absolute_time();
		}
		
		for(i = 0;i < 3;i++)
		{
			sensors.gyro_raw[i] = gyro_raw[i];
			sensors.gyro_rad_s[i] = (gyro_raw[i] - gyro_offset[i] * IMU_DEG_PER_LSB_CFG);
			sensors.gyro_timestamp = hrt_absolute_time();
		}
		
		if(mag_counter >= mag_div)
		{
			mag_counter = 0;
			for(i = 0;i < 3;i++)
			{
				sensors.mag_raw[i]  = mag_raw[i];
				sensors.mag_ga[i]     = (mag_raw[i] - mag_offset[i] * IMU_1G_RAW);
				sensors.mag_timestamp = hrt_absolute_time();
			}
		}
		else
		{
			mag_counter++;
		}
		
		for(i = 0;i < 3;i++)
		{
			sensors.acc_m_s2[i] = KalmanFilter(&acc_filter[i],sensors.acc_m_s2[i]);
			sensors.mag_ga[i] = KalmanFilter(&mag_filter[i],sensors.mag_ga[i]);
		}
		orb_publish(ORB_ID(sensor_combined),&sensors);
		
		rt_thread_delay(2);
	}
}

