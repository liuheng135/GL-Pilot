#include "spi.h"
#include <rthw.h>
#include <rtthread.h>
#include "board.h"

struct spi_bus spi1 =
{
    SPI1,
    {
		SPI_Mode_Master,
		SPI_Direction_2Lines_FullDuplex,
		20000000,
		SPI_DataSize_8b,
		SPI_CPOL_High,
		SPI_CPHA_2Edge,
		SPI_FirstBit_MSB,
	},
    "spi1",
	RT_NULL,
	RT_NULL,
	spi_transfer,
};
struct rt_device spi1_device;




void spi_configure(struct spi_bus  *bus)
{
	SPI_InitTypeDef SPI_InitStructure; 
	
	SPI_InitStructure.SPI_Direction = bus->_config.direction; //????? 
	SPI_InitStructure.SPI_Mode = bus->_config.mode; //??? 
	SPI_InitStructure.SPI_DataSize = bus->_config.datasize; //????8? 
	SPI_InitStructure.SPI_CPOL = bus->_config.cpol;  // Transitioned On The Falling Edge
	SPI_InitStructure.SPI_CPHA = bus->_config.cpha; // Latched On the Rising Edge
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS??????? 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;// fsck = APB2 84MHz / 4 = 21MHz
	SPI_InitStructure.SPI_FirstBit = bus->_config.firstbit; //???? 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(bus->_bus, &SPI_InitStructure); 
}
void spi_hw_init(struct spi_bus* bus)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	
	if(bus->_bus == SPI1)
	{
		RCC_AHB1PeriphClockCmd(SPI1_GPIO_RCC, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
		
		GPIO_PinAFConfig(SPI1_GPIO_PORT, SPI1_PINSOURCE_SCK, GPIO_AF_SPI1);
		GPIO_PinAFConfig(SPI1_GPIO_PORT, SPI1_PINSOURCE_MISO, GPIO_AF_SPI1);	
		GPIO_PinAFConfig(SPI1_GPIO_PORT, SPI1_PINSOURCE_MOSI, GPIO_AF_SPI1);	
		
		GPIO_InitStructure.GPIO_Pin = SPI1_GPIO_SCK|SPI1_GPIO_MISO|SPI1_GPIO_MOSI; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;   
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStructure);			
	}
	
	spi_configure(bus);
	//SPI_Cmd(SPIx, ENABLE);
}

//spi 读写一个字节
rt_err_t spi_transfer(SPI_TypeDef* SPIx, uint8_t *txdata,uint8_t *rxdata)
{		
	u8 retry=0;				 
	while((SPIx->SR&1<<1)==0)//等待发送区空	
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPIx->DR=*txdata;	 	  //发送一个byte 
	retry=0;
	while((SPIx->SR&1<<0)==0) //等待接收完一个byte  
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	*rxdata = SPIx->DR;          //返回收到的数据	
    return RT_EOK;	
} 

//设置SPI的速度
rt_err_t spi_setspeed(SPI_TypeDef* SPIx, uint32_t max_hz)
{
	uint32_t SPI_APB_CLOCK;
	uint32_t stm32_spi_max_clock;
	
	SPIx->CR1&=0XFFC7;

	if(SPIx == SPI1)
	{
		stm32_spi_max_clock = 42000000;
		SPI_APB_CLOCK = SystemCoreClock / 2;
	}
	else
	{
		stm32_spi_max_clock = 24000000;
		SPI_APB_CLOCK = SystemCoreClock / 4;
	}

	if(max_hz > stm32_spi_max_clock)
	{
		max_hz = stm32_spi_max_clock;
	}

	/* STM32F4xx SPI MAX 42Mhz */
	if(max_hz >= SPI_APB_CLOCK/2)
	{
		SPIx->CR1 |= SPI_BaudRatePrescaler_2;
	}
	else if(max_hz >= SPI_APB_CLOCK/4)
	{
		SPIx->CR1 |= SPI_BaudRatePrescaler_4;
	}
	else if(max_hz >= SPI_APB_CLOCK/8)
	{
		SPIx->CR1 |= SPI_BaudRatePrescaler_8;
	}
	else if(max_hz >= SPI_APB_CLOCK/16)
	{
		SPIx->CR1 |= SPI_BaudRatePrescaler_16;
	}
	else if(max_hz >= SPI_APB_CLOCK/32)
	{
		SPIx->CR1 |= SPI_BaudRatePrescaler_32;
	}
	else if(max_hz >= SPI_APB_CLOCK/64)
	{
		SPIx->CR1 |= SPI_BaudRatePrescaler_64;
	}
	else if(max_hz >= SPI_APB_CLOCK/128)
	{
		SPIx->CR1 |= SPI_BaudRatePrescaler_128;
	}
	else
	{
		/*  min prescaler 256 */
		SPIx->CR1 |= SPI_BaudRatePrescaler_256;
	}
		
	SPIx->CR1|=1<<6; //SPI设备使能
	return RT_EOK;
} 

static rt_err_t spi_init (rt_device_t dev)
{
	struct rt_mutex *mutex;
    struct spi_bus* spi = (struct spi_bus*) dev->user_data;

	mutex = (rt_mutex_t)rt_malloc(sizeof(struct rt_mutex));
	rt_mutex_init(mutex,spi->_name,RT_IPC_FLAG_FIFO);
	/*if dev is not activated*/
    if (!(dev->flag & RT_DEVICE_FLAG_ACTIVATED))
    {
		/*initialize hardware*/
		spi_hw_init(spi);
		/*if dev is in int rx mode*/
        if (dev->flag & RT_DEVICE_FLAG_INT_RX)
        {
            /* not support */
        }
        /* Enable USART */
        SPI_Cmd(spi->_bus, ENABLE);

        dev->flag |= RT_DEVICE_FLAG_ACTIVATED;
    }
	spi->_spi_mutex = mutex;
    return RT_EOK;
}

static rt_err_t spi_open(rt_device_t dev, uint16_t oflag)
{
	rt_err_t err;
    struct spi_bus* spi = (struct spi_bus*) dev->user_data;
	
	err = rt_mutex_take(spi->_spi_mutex,2);

	return err;
}

static rt_err_t spi_close(rt_device_t dev)
{
    rt_err_t err;
    struct spi_bus* spi = (struct spi_bus*) dev->user_data;
	
	err = rt_mutex_release(spi->_spi_mutex);

	return err;
}

static rt_size_t spi_read (rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    rt_uint8_t* ptr;
	rt_uint8_t  tx;
    rt_err_t err_code;
    struct spi_bus* spi;

    ptr = buffer;
    err_code = RT_EOK;
    spi = (struct spi_bus*)dev->user_data;

	if (dev->flag & RT_DEVICE_FLAG_INT_TX)
    {
        /* interrupt mode Tx, does not support */
        RT_ASSERT(0);
    }
    else if (dev->flag & RT_DEVICE_FLAG_DMA_TX)
    {
		/* DMA mode Tx, does not support */
        RT_ASSERT(0);
    }
    else
    {
		/* polling mode */
		rt_base_t level;
        level = rt_hw_interrupt_disable();
        while (size--)
        {
            spi_transfer(spi->_bus,&tx,ptr);
			ptr++;			
        }
		rt_hw_interrupt_enable(level);
    }

    /* set error code */
    rt_set_errno(err_code);
    return (rt_uint32_t)ptr - (rt_uint32_t)buffer;
}

static rt_size_t spi_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    rt_uint8_t* ptr;
	rt_uint8_t rx;
    rt_err_t err_code;
    struct spi_bus* spi;

    err_code = RT_EOK;
    ptr = (rt_uint8_t*)buffer;
    spi = (struct spi_bus*)dev->user_data;

    if (dev->flag & RT_DEVICE_FLAG_INT_TX)
    {
        /* interrupt mode Tx, does not support */
        RT_ASSERT(0);
    }
    else if (dev->flag & RT_DEVICE_FLAG_DMA_TX)
    {
		/* DMA mode Tx, does not support */
        RT_ASSERT(0);
    }
    else
    {
        /* polling mode */
        rt_base_t level;
        level = rt_hw_interrupt_disable();
        while (size--)
        {
            spi_transfer(spi->_bus,ptr,&rx);
            ptr++;			
        }
		rt_hw_interrupt_enable(level);
    }

    /* set error code */
    rt_set_errno(err_code);

    return (rt_uint32_t)ptr - (rt_uint32_t)buffer;
}

static rt_err_t spi_control (rt_device_t dev, rt_uint8_t cmd, void *args)
{
    struct spi_bus* spi;
	uint32_t* speed = args;
    RT_ASSERT(dev != RT_NULL);

    spi = (struct spi_bus*)dev->user_data;
    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        /* suspend device */
        dev->flag |= RT_DEVICE_FLAG_SUSPENDED;
        SPI_Cmd(spi->_bus, DISABLE);
        break;

    case RT_DEVICE_CTRL_RESUME:
        /* resume device */
        dev->flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        SPI_Cmd(spi->_bus, ENABLE);
        break;
	
	case RT_DEVICE_CTRL_CONFIG:
		spi_configure(spi);
	    break;
	case RT_DEVICE_CTRL_SPEED:
		spi_setspeed(spi->_bus,*speed);
    }  

    return RT_EOK;
}

rt_err_t spi_register(rt_device_t device, const char* name, rt_uint32_t flag, struct spi_bus *bus)
{
    RT_ASSERT(device != RT_NULL);

    if ((flag & RT_DEVICE_FLAG_DMA_RX) ||
        (flag & RT_DEVICE_FLAG_INT_TX))
    {
        RT_ASSERT(0);
    }

    device->type        = RT_Device_Class_SPIBUS;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->init        = spi_init;
    device->open        = spi_open;
    device->close       = spi_close;
    device->read        = spi_read;
    device->write       = spi_write;
    device->control     = spi_control;
    device->user_data   = bus;

    /* register a character device */
    return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR | flag);
}

void  hw_spi_init(void)
{
	spi_register(&spi1_device, spi1._name,
        RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
        &spi1);
}
