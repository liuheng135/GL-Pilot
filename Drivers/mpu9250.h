#ifndef __MPU9250_H_
#define __MPU9250_H_

#include <stdint.h>
#include <rtthread.h>

#define MPU9250_CSM_PORT              GPIOC
#define MPU9250_CSM_CLK               RCC_AHB1Periph_GPIOC 
#define MPU9250_CSM_PIN               GPIO_Pin_4
#define Set_MPU9250_CSM  MPU9250_CSM_PORT->BSRRL = MPU9250_CSM_PIN //{GPIO_SetBits(MPU9250_CSM_PORT,MPU9250_CSM_PIN);}
#define Clr_MPU9250_CSM  MPU9250_CSM_PORT->BSRRH = MPU9250_CSM_PIN //{GPIO_ResetBits(MPU9250_CSM_PORT,MPU9250_CSM_PIN);}  
//INT主机数据输入
#define MPU9250_INT_PORT              GPIOA
#define MPU9250_INT_CLK               RCC_AHB1Periph_GPIOA  
#define MPU9250_INT_PIN               GPIO_Pin_4 
#define Set_MPU9250_INT  MPU9250_INT_PORT->BSRRL = MPU9250_INT_PIN//{GPIO_SetBits(MPU9250_INT_PORT,MPU9250_INT_PIN);}
#define Clr_MPU9250_INT  MPU9250_INT_PORT->BSRRH = MPU9250_INT_PIN{GPIO_ResetBits(MPU9250_INT_PORT,MPU9250_INT_PIN);} 
#define MPU9250_INT  (GPIO_ReadInputDataBit(MPU9250_INT_PORT, MPU9250_INT_PIN))



#define MPU6500_DEG_PER_LSB_250  0.0076295109483482f  //((2 * 250.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_500  0.0152587890625f     //((2 * 500.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_1000 0.030517578125f      //((2 * 1000.0) / 65536.0)
#define MPU6500_DEG_PER_LSB_2000 0.06103515625f       //((2 * 2000.0) / 65536.0)

#define MPU6500_G_PER_LSB_2      0.00006103515625f    //((2 * 2) / 65536.0)
#define MPU6500_G_PER_LSB_4      0.0001220703125f     //((2 * 4) / 65536.0)
#define MPU6500_G_PER_LSB_8      0.000244140625f     //((2 * 8) / 65536.0)
#define MPU6500_G_PER_LSB_16     0.00048828125f       //((2 * 16) / 65536.0)

#define AK8963_uT_PER_LSB_4900   0.15f     //16bit

#define MPU6500_ST_GYRO_LOW      10.0   // deg/s
#define MPU6500_ST_GYRO_HIGH     105.0  // deg/s
#define MPU6500_ST_ACCEL_LOW     0.300  // G
#define MPU6500_ST_ACCEL_HIGH    0.950  // G


typedef struct
{
    int16_t x;
    int16_t y;
	int16_t z;
}int_3_axis;

typedef struct
{
    float x;
    float y;
	float z;
}float_3_axis;



rt_err_t MPU9250_Init(void);

void MPU9250_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, 
						int16_t* gx, int16_t* gy, int16_t* gz);

void MPU9250_GetMotion9(int16_t *ax, int16_t *ay, int16_t *az, 
						int16_t *gx, int16_t *gy, int16_t *gz, 
						int16_t *mx, int16_t *my, int16_t *mz);


#endif
