#ifndef __SPI_H_
#define __SPI_H_

#include "stm32f4xx.h"
#include "rtthread.h"

#define SPI1_GPIO_RCC       RCC_AHB1Periph_GPIOA
#define SPI1_GPIO_PORT      GPIOA
#define SPI1_GPIO_SCK       GPIO_Pin_5
#define SPI1_GPIO_MISO      GPIO_Pin_6
#define SPI1_GPIO_MOSI      GPIO_Pin_7
#define SPI1_PINSOURCE_SCK  GPIO_PinSource5 
#define SPI1_PINSOURCE_MISO GPIO_PinSource6 
#define SPI1_PINSOURCE_MOSI GPIO_PinSource7 



struct spi_config_struct
{
	uint32_t mode;
    uint32_t direction;
	uint32_t maxspeed;
	uint32_t datasize;
	uint32_t cpol;
	uint32_t cpha;
	uint32_t firstbit;
};

struct spi_bus
{
	SPI_TypeDef*               _bus;
    struct spi_config_struct   _config;
	char*                      _name;
	rt_device_t*               _owner;
    struct rt_mutex*           _spi_mutex;
    rt_err_t  (*transfer)      (SPI_TypeDef* SPIx, uint8_t *txdata,uint8_t *rxdata);
};

void  hw_spi_init(void);
rt_err_t spi_transfer(SPI_TypeDef* SPIx, uint8_t *txdata,uint8_t *rxdata);
rt_err_t spi_setspeed(SPI_TypeDef* SPIx, uint32_t max_hz);
#endif
