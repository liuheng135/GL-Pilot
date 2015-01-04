#ifndef __MYSERIAL_H__
#define __MYSERIAL_H__

#include "stm32f4xx.h"
#include "rtthread.h"

#define UART_RX_BUFFER_SIZE		64


#define USART1_GPIO_PORT            GPIOA
#define USART1_GPIO_RCC             RCC_AHB1Periph_GPIOA
#define USART1_GPIO_TX              GPIO_Pin_9
#define USART1_GPIO_RX              GPIO_Pin_10
#define USART1_GPIO_PINSOURCE_TX    GPIO_PinSource9
#define USART1_GPIO_PINSOURCE_RX    GPIO_PinSource10

#define USART2_GPIO_PORT            GPIOA
#define USART2_GPIO_RCC             RCC_AHB1Periph_GPIOA
#define USART2_GPIO_TX              GPIO_Pin_2
#define USART2_GPIO_RX              GPIO_Pin_3
#define USART2_GPIO_PINSOURCE_TX    GPIO_PinSource2
#define USART2_GPIO_PINSOURCE_RX    GPIO_PinSource3

#define USART3_GPIO_PORT            GPIOB
#define USART3_GPIO_RCC             RCC_AHB1Periph_GPIOB
#define USART3_GPIO_TX              GPIO_Pin_10
#define USART3_GPIO_RX              GPIO_Pin_11
#define USART3_GPIO_PINSOURCE_TX    GPIO_PinSource10
#define USART3_GPIO_PINSOURCE_RX    GPIO_PinSource11

#define UART4_GPIO_PORT             GPIOC
#define UART4_GPIO_RCC              RCC_AHB1Periph_GPIOC
#define UART4_GPIO_TX               GPIO_Pin_10
#define UART4_GPIO_RX               GPIO_Pin_11
#define UART4_GPIO_PINSOURCE_TX     GPIO_PinSource10
#define UART4_GPIO_PINSOURCE_RX     GPIO_PinSource11

#define USART_CTRL_BASE              0x03
#define USART_CTRL_BAUDRATE         (USART_CTRL_BASE+0)
#define USART_CTRL_STOPBIT          (USART_CTRL_BASE+1)
#define USART_CTRL_CONFIG           (USART_CTRL_BASE+2)

struct serial_int_tx
{
	uint8_t  tx_buffer[UART_RX_BUFFER_SIZE];
	uint32_t read_index, save_index;
};

struct serial_int_rx
{
	uint8_t  rx_buffer[UART_RX_BUFFER_SIZE];
	uint32_t read_index, save_index;
};

struct serial_device
{
	USART_TypeDef* uart_device;
	
	uint32_t baudrate;
	uint32_t wordlength;
	uint32_t stopbit;
	uint32_t Parity;
	

	/* rx structure */
	struct serial_int_rx* int_rx;

	/* tx structure */
	struct serial_int_tx* dma_tx;
	char *name;
	
};

extern struct rt_device usart1_device;
extern struct rt_device usart2_device;
extern struct rt_device usart3_device;
extern struct rt_device uart4_device;

void hw_usart_init(void);
void  hw_serial_isr(rt_device_t device);



#endif
