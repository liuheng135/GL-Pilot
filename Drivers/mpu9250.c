#include "mpu9250.h"
#include "rtthread.h"
#include "stm32f4xx.h"
#include <math.h>
#include <stdbool.h>

static rt_device_t mpu9250_spi = RT_NULL;

#define    MPU9250_GET_TIME_MS()           hrt_absolute_ms()
#define    MPU9250_BUS                     "spi1"
	
void MPU9250_SPI_WRITE(uint8_t dat)
{
	rt_device_write(mpu9250_spi,0, &dat,1);
}

uint8_t MPU9250_SPI_READ(void)
{
	uint8_t dat;
	rt_device_read(mpu9250_spi,0,&dat,1);
	return dat;
}


#define DEBUG_PRINT(...)

static void MPU9250_DelayMs(uint32_t ms)
{
	rt_thread_delay(ms);	
}

static void MPU9250_PinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(MPU9250_CSM_CLK | MPU9250_INT_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = MPU9250_CSM_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(MPU9250_CSM_PORT, &GPIO_InitStructure);
	Set_MPU9250_CSM;	 
	 

	GPIO_InitStructure.GPIO_Pin = MPU9250_INT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN  ;   
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(MPU9250_INT_PORT, &GPIO_InitStructure);
}	

static void MPU9250_ReadReg(uint8_t reg, uint8_t *rddata)
{
	Clr_MPU9250_CSM;
	MPU9250_SPI_WRITE(0x80 | reg);
	*rddata = MPU9250_SPI_READ();
	Set_MPU9250_CSM;
}

static void MPU9250_ReadMulti(uint8_t reg, uint8_t *buff, uint8_t num)
{
	uint8_t i;
	
	Clr_MPU9250_CSM;
	MPU9250_SPI_WRITE(0x80 | reg);
	for(i=0; i<num; i++)
		*buff++ = MPU9250_SPI_READ();
	Set_MPU9250_CSM;
}

static void MPU9250_WriteReg(uint8_t reg, uint8_t wrdata)
{
	Clr_MPU9250_CSM;
	MPU9250_SPI_WRITE(reg);
	MPU9250_SPI_WRITE(wrdata);
	Set_MPU9250_CSM;
	MPU9250_DelayMs(2);
}

static float MPU9250_GetFullScaleGyroDPL(void)
{
	uint8_t data;
	float range;
	
	MPU9250_ReadReg(27, &data);
	data >>= 3;
	switch(data)
	{
		case 0:
			range = MPU6500_DEG_PER_LSB_250;
			break;
		case 1:
			range = MPU6500_DEG_PER_LSB_500;
			break;
		case 2:
			range = MPU6500_DEG_PER_LSB_1000;
			break;
		case 3:
			range = MPU6500_DEG_PER_LSB_2000;
			break;
		default:
			range = MPU6500_DEG_PER_LSB_1000;
			break;
	}
	
	return range;
}
static float MPU9250_GetFullScaleAccelGPL(void)
{
	uint8_t data;
	float range;
	
	MPU9250_ReadReg(28, &data);
	data >>= 3;
	switch(data)
	{
		case 0:
			range = MPU6500_G_PER_LSB_2;
			break;
		case 1:
			range = MPU6500_G_PER_LSB_4;
			break;
		case 2:
			range = MPU6500_G_PER_LSB_8;
			break;
		case 3:
			range = MPU6500_G_PER_LSB_16;
			break;
		default:
			range = MPU6500_G_PER_LSB_8;
			break;
	}
	
	return range;
}

static void MPU9250_AK8963_WriteReg(uint8_t reg, uint8_t wrdata)
{
	MPU9250_WriteReg(37, 0X0C);//AK8975 Address WRITE
	MPU9250_WriteReg(38, reg);//AK8975 Reg Address
	MPU9250_WriteReg(39, 0x81);//enable
	MPU9250_WriteReg(99, wrdata);//data
	MPU9250_DelayMs(10);
	MPU9250_WriteReg(39, 0x00);//disable
	
}
static void MPU9250_AK8963_GetAdjustment(int8_t *x, int8_t *y, int8_t *z)
{
	int8_t buff[3];
	
	MPU9250_AK8963_WriteReg(0X0A, 0X0F);//Fuse ROM access mode 
	
	MPU9250_WriteReg(37, 0X8C);//AK8975 Address READ
	MPU9250_WriteReg(38, 0X10);//AK8975 ASAX
	MPU9250_WriteReg(39, 0x83);//enable
	
	MPU9250_DelayMs(10);
	MPU9250_WriteReg(39, 0x00);//disable
	
	MPU9250_ReadMulti(73, (uint8_t*)buff, 3);
	*x = buff[0];
	*y = buff[1];
	*z = buff[2];
	
	MPU9250_AK8963_WriteReg(0X0A, 0X00);//Power-down mode
}
static void MPU9250_AK8963_ReadData(int16_t *mx, int16_t *my, int16_t *mz)
{
	uint8_t buff[6];
	
	MPU9250_WriteReg(37, 0X8C);//AK8975 Address READ
	MPU9250_WriteReg(38, 0X03);//AK8975 HXL
	MPU9250_WriteReg(39, 0x86);//enable
	
	MPU9250_DelayMs(10);
	MPU9250_WriteReg(39, 0x00);//disable
	
	MPU9250_ReadMulti(73, buff, 6);
    *mx = (((int16_t)buff[1]) << 8) | buff[0];
    *my = (((int16_t)buff[3]) << 8) | buff[2];
    *mz = (((int16_t)buff[5]) << 8) | buff[4];
}

//检测AK8963是否存在
//返回值:0，成功;1，失败	
static uint8_t MPU9250_AK8963_Check(void)
{
	uint8_t aid=0;
	
	MPU9250_WriteReg(37, 0X8C);//AK8975 Address READ
	MPU9250_WriteReg(38, 0x00);//AK8975 WIA
	MPU9250_WriteReg(39, 0x81);//enable	
	MPU9250_DelayMs(10);
	MPU9250_WriteReg(39, 0x00);//disable
	MPU9250_ReadReg(73, &aid);
	if(aid != 0x48)
		return 1;
	else
		return 0;
}

//返回值:0，成功;1，失败	
static uint8_t MPU9250_AK8963_SelfTest(void)
{
	int16_t smx, smy, smz;
	
	if(MPU9250_AK8963_Check())
	{
		return 1;
	}
	MPU9250_AK8963_WriteReg(0X0A, 0X00);//Power-down mode
	MPU9250_AK8963_WriteReg(0X0C, 0X40);//SELF TEST
	MPU9250_AK8963_WriteReg(0X0A, 0X18);//16BIT Self-test Mode
	
	MPU9250_DelayMs(100);
	MPU9250_AK8963_ReadData(&smx, &smy, &smz);
	
	MPU9250_AK8963_WriteReg(0X0C, 0X00);
	MPU9250_AK8963_WriteReg(0X0A, 0X00);//Power-down mode
	
	if(smx < -200 || smx > 200)
		return 1;
	if(smy < -200 || smy > 200)
		return 1;
	if(smz < -3200 || smz > -800)
		return 1;
	
	return 0;
}

//MPU9250设置电子罗盘连续测量自动读取
void MPU9250_MagMeasureContinu(void)
{
	//SLV0
	MPU9250_WriteReg(37, 0X8C);//AK8975 Address READ
	MPU9250_WriteReg(38, 0X03);//AK8975 HXL
	MPU9250_WriteReg(39, 0x86);//enable		

	//SLV1
	MPU9250_WriteReg(40, 0X0C);//AK8975 Address READ
	MPU9250_WriteReg(41, 0X0A);//AK8975 CNTL1
	MPU9250_WriteReg(42, 0x81);//enable	
	MPU9250_WriteReg(100, 0X11);//16BIT Single measurement mode 
	
	MPU9250_WriteReg(52, 0x04);//I2C_MST_DLY = 4
	MPU9250_WriteReg(103, 0x03);//I2C_SLV0_DLY_EN 
}


//检测MPU9250是否存在
//返回值:0，成功;1，失败	
uint8_t MPU9250_Check(void)
{
	uint8_t mid = 0;
	
	MPU9250_ReadReg(117, &mid);
	if(mid == 0x71)
		return 0;
	else
		return 1;
}

static bool MPU9250_EvaluateSelfTest(float low, float high, float value, char* string)
{
	if (value < low || value > high)
	{
		return false;
	}
	return true;
}

//返回值:0，成功;1，失败	
static uint8_t MPU9250_SelfTest(void)
{
	uint8_t asel,gsel;
	
	int16_t axi16, ayi16, azi16;
	int16_t gxi16, gyi16, gzi16;
	float axf, ayf, azf;
	float gxf, gyf, gzf;
	float axfTst, ayfTst, azfTst;
	float gxfTst, gyfTst, gzfTst;
	float axfDiff, ayfDiff, azfDiff;
	float gxfDiff, gyfDiff, gzfDiff;
	float gRange, aRange;
	uint32_t scrap;
	
	rt_device_open(mpu9250_spi,RT_DEVICE_OFLAG_RDWR);
	aRange = MPU9250_GetFullScaleAccelGPL();
	gRange = MPU9250_GetFullScaleGyroDPL();

	// First values after startup can be read as zero. Scrap a couple to be sure.
	for (scrap = 0; scrap < 20; scrap++)
	{
		MPU9250_GetMotion6(&axi16, &ayi16, &azi16, &gxi16, &gyi16, &gzi16);
		MPU9250_DelayMs(2);
	}
	// First measurement
	gxf = gxi16 * gRange;
	gyf = gyi16 * gRange;
	gzf = gzi16 * gRange;
	axf = axi16 * aRange;
	ayf = ayi16 * aRange;
	azf = azi16 * aRange;

	// Enable self test
	MPU9250_ReadReg(27, &gsel);
	MPU9250_ReadReg(28, &asel);
	MPU9250_WriteReg(27, 0xE0|gsel);
	MPU9250_WriteReg(28, 0xE0|asel);


	// Wait for self test to take effect
	MPU9250_DelayMs(10);
	// Take second measurement
	MPU9250_GetMotion6(&axi16, &ayi16, &azi16, &gxi16, &gyi16, &gzi16);
	gxfTst = gxi16 * gRange;
	gyfTst = gyi16 * gRange;
	gzfTst = gzi16 * gRange;
	axfTst = axi16 * aRange;
	ayfTst = ayi16 * aRange;
	azfTst = azi16 * aRange;

	// Disable self test
	MPU9250_WriteReg(27, gsel);
	MPU9250_WriteReg(28, asel);

	// Calculate difference
	gxfDiff = gxfTst - gxf;
	gyfDiff = gyfTst - gyf;
	gzfDiff = gzfTst - gzf;
	axfDiff = axfTst - axf;
	ayfDiff = ayfTst - ayf;
	azfDiff = azfTst - azf;

	// Check result
	if (MPU9250_EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gxfDiff, "gyro X") &&
	MPU9250_EvaluateSelfTest(-MPU6500_ST_GYRO_HIGH, -MPU6500_ST_GYRO_LOW, gyfDiff, "gyro Y") &&
	MPU9250_EvaluateSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gzfDiff, "gyro Z") &&
	MPU9250_EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, axfDiff, "acc X") &&
	MPU9250_EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, ayfDiff, "acc Y") &&
	MPU9250_EvaluateSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, azfDiff, "acc Z"))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}



rt_err_t MPU9250_Init(void)
{
	uint32_t speed  = 100000;
	mpu9250_spi = rt_device_find(MPU9250_BUS);
	rt_device_open(mpu9250_spi,RT_DEVICE_OFLAG_RDWR);
	
	rt_device_control(mpu9250_spi,RT_DEVICE_CTRL_SPEED,&speed);
	speed = 20000000;
	
	MPU9250_PinInit(); 
	
	MPU9250_WriteReg(107, 0X80);//Reset
	MPU9250_DelayMs(100);
	MPU9250_WriteReg(107, 0X01);//Clock Source 
	MPU9250_WriteReg(108, 0X00);//Enable Acc & Gyro
	
	MPU9250_WriteReg(56, 0X01);//enabled RAW_RDY_EN Interrupt
	MPU9250_WriteReg(55, 0XB0);//disabled BYPASS  LOW INT
	
	MPU9250_WriteReg(106, 0X30);//I2C_MST_EN
	MPU9250_WriteReg(36, 0X4D);//I2C Speed 400 kHz
	
	MPU9250_WriteReg(25, 0X01);//SMPLRT_DIV
	MPU9250_WriteReg(26, 0X01);//Bandwidth = 184Hz, FS=1KHz
	MPU9250_WriteReg(27, 0X18);//2000 dps 
	MPU9250_WriteReg(28, 0X10);//8g 
	MPU9250_WriteReg(29, 0X00);//Bandwidth = 460Hz, FS=1KHz
	
	MPU9250_AK8963_WriteReg(0x0b, 0x01);//Reset AK8963
	
	
	if(MPU9250_AK8963_SelfTest())
	{
		rt_device_control(mpu9250_spi,RT_DEVICE_CTRL_SPEED,&speed);
		rt_device_close(mpu9250_spi);
	    return RT_ERROR;
	}
	if(MPU9250_SelfTest())
	{
		rt_device_control(mpu9250_spi,RT_DEVICE_CTRL_SPEED,&speed);
		rt_device_close(mpu9250_spi);
	    return RT_ERROR;
	}
	rt_device_control(mpu9250_spi,RT_DEVICE_CTRL_SPEED,&speed);
	rt_device_close(mpu9250_spi);
	return RT_EOK;
}

void MPU9250_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, 
						int16_t* gx, int16_t* gy, int16_t* gz)
{
	
	uint8_t buffer[14];
	
	rt_device_open(mpu9250_spi,RT_DEVICE_OFLAG_RDWR);
	MPU9250_ReadMulti(59, buffer, 14);
	*ax = (((int16_t) buffer[0]) << 8) | buffer[1];
	*ay = (((int16_t) buffer[2]) << 8) | buffer[3];
	*az = (((int16_t) buffer[4]) << 8) | buffer[5];
	*gx = (((int16_t) buffer[8]) << 8) | buffer[9];
	*gy = (((int16_t) buffer[10]) << 8) | buffer[11];
	*gz = (((int16_t) buffer[12]) << 8) | buffer[13];
	rt_device_close(mpu9250_spi);
}


void MPU9250_GetMotion9(int16_t *ax, int16_t *ay, int16_t *az, 
						int16_t *gx, int16_t *gy, int16_t *gz, 
						int16_t *mx, int16_t *my, int16_t *mz)
{
	uint8_t buffer[20];
	
    rt_device_open(mpu9250_spi,RT_DEVICE_OFLAG_RDWR);
	MPU9250_ReadMulti(59, buffer, 20);
	*ax = (((int16_t) buffer[0]) << 8) | buffer[1];
	*ay = (((int16_t) buffer[2]) << 8) | buffer[3];
	*az = (((int16_t) buffer[4]) << 8) | buffer[5];
	
	*gx = (((int16_t) buffer[8]) << 8) | buffer[9];
	*gy = (((int16_t) buffer[10]) << 8) | buffer[11];
	*gz = (((int16_t) buffer[12]) << 8) | buffer[13];
	
	*mx = (((int16_t) buffer[15]) << 8) | buffer[14];
	*my = (((int16_t) buffer[17]) << 8) | buffer[16];
	*mz = (((int16_t) buffer[19]) << 8) | buffer[18];
	rt_device_close(mpu9250_spi);
}



