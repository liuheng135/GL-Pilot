#include "app_mpu9250.h"

void app_mpu9250_main()
{
	uint32_t i;
	int16_t acc_raw[3];
	int16_t gyro_raw[3];
	int16_t mag_raw[3];
	
	int16_t acc_offset[3] = {0};
	int16_t gyro_offset[3] = {0};
	int16_t mag_offset[3] = {0};
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
	while(1)
	{
		rt_thread_delay(2);
		MPU9250_GetMotion9(&acc_raw[0],
		                   &acc_raw[1],
		                   &acc_raw[2],
		                   &gyro_raw[0],
		                   &gyro_raw[1],
		                   &gyro_raw[2],
		                   &mag_raw[0],
		                   &mag_raw[1],
		                   &mag_raw[2]);
		
		/* we can do rotation here if we need*/
		
		for(i = 0;i < 3;i++)
		{
			sensors.acc_raw[i]  = acc_raw[i];
			sensors.gyro_raw[i] = gyro_raw[i];
			sensors.mag_raw[i]  = mag_raw[i];
			
			sensors.acc_m_s2[i]   = (acc_raw[i] - acc_offset[i]) * IMU_G_PER_LSB_CFG * G;
			sensors.gyro_rad_s[i] = (gyro_raw[i] - gyro_offset[i] * IMU_DEG_PER_LSB_CFG);
			sensors.mag_ga[i]     = (mag_raw[i] - mag_offset[i] * IMU_1G_RAW);
		}
		
		for(i = 0;i < 3;i++)
		{
			sensors.acc_m_s2[i] = KalmanFilter(&acc_filter[i],sensors.acc_m_s2[i]);
			sensors.mag_ga[i] = KalmanFilter(&mag_filter[i],sensors.mag_ga[i]);
		}
		
		orb_publish(ORB_ID(sensor_combined),&sensors);
	}
}