#include "serial.h"
#include <rthw.h>
#include <rtthread.h>
#include "board.h"

struct  serial_int_rx usart1_int_rx;
struct serial_device usart1 =
{
    USART1,
	115200,
	USART_WordLength_8b,
	USART_StopBits_1,
	USART_Parity_No,
    &usart1_int_rx,
    RT_NULL,
	"usart1"
};
struct rt_device usart1_device;


struct serial_int_rx usart2_int_rx;

struct serial_device usart2 =
{
    USART2,
	115200,
	USART_WordLength_8b,
	USART_StopBits_1,
	USART_Parity_No,
    &usart2_int_rx,
    RT_NULL,
	"usart2"
};
struct rt_device usart2_device;

struct  serial_int_rx usart3_int_rx;
struct serial_device usart3 =
{
    USART2,
	115200,
	USART_WordLength_8b,
	USART_StopBits_1,
	USART_Parity_No,
    &usart3_int_rx,
    RT_NULL,
	"usart3"
};
struct rt_device usart3_device;

struct  serial_int_rx uart4_int_rx;
struct serial_device uart4 =
{
    USART2,
	115200,
	USART_WordLength_8b,
	USART_StopBits_1,
	USART_Parity_No,
    &uart4_int_rx,
    RT_NULL,
	"uart4"
};
struct rt_device uart4_device;

static void uasrt_nvic_configuration(USART_TypeDef* usart)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    if(usart == USART1)
	{
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	}
	else if(usart == USART2)
	{
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	}
	
	else if(usart == USART3)
	{
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	}
	
	else if(usart == UART4)
	{
		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	}
	else 
	{
		return;
	}
	NVIC_Init(&NVIC_InitStructure);
	
}

static void serial_config(struct serial_device *dev)
{
	
	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = dev->baudrate;
	USART_InitStructure.USART_WordLength = dev->wordlength;
	USART_InitStructure.USART_StopBits = dev->stopbit;
	USART_InitStructure.USART_Parity = dev->Parity;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(dev->uart_device, &USART_InitStructure);
}


void uasrt_hw_init(struct serial_device *dev)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	if(dev->uart_device == USART1)
	{
		RCC_AHB1PeriphClockCmd(USART1_GPIO_RCC, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin = USART1_GPIO_TX | USART1_GPIO_RX;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(USART1_GPIO_PORT, &GPIO_InitStructure);
		
		GPIO_PinAFConfig(USART1_GPIO_PORT,USART1_GPIO_PINSOURCE_TX,GPIO_AF_USART1);
		GPIO_PinAFConfig(USART1_GPIO_PORT,USART1_GPIO_PINSOURCE_RX,GPIO_AF_USART1);
		
	}
	else if(dev->uart_device == USART2)
	{
		RCC_AHB1PeriphClockCmd(USART2_GPIO_RCC, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin = USART2_GPIO_TX | USART2_GPIO_RX;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(USART2_GPIO_PORT, &GPIO_InitStructure);
		
		GPIO_PinAFConfig(USART2_GPIO_PORT,USART2_GPIO_PINSOURCE_TX,GPIO_AF_USART2);
		GPIO_PinAFConfig(USART2_GPIO_PORT,USART2_GPIO_PINSOURCE_RX,GPIO_AF_USART2);
	}
	else if(dev->uart_device == USART3)
	{
		RCC_AHB1PeriphClockCmd(USART3_GPIO_RCC, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin = USART3_GPIO_TX | USART3_GPIO_RX;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(USART3_GPIO_PORT, &GPIO_InitStructure);
		
		GPIO_PinAFConfig(USART3_GPIO_PORT,USART3_GPIO_PINSOURCE_TX,GPIO_AF_USART3);
		GPIO_PinAFConfig(USART3_GPIO_PORT,USART3_GPIO_PINSOURCE_RX,GPIO_AF_USART3);
	}
	else if(dev->uart_device == UART4)
	{
		RCC_AHB1PeriphClockCmd(UART4_GPIO_RCC, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin = UART4_GPIO_TX | UART4_GPIO_RX;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(UART4_GPIO_PORT, &GPIO_InitStructure);
		
		GPIO_PinAFConfig(UART4_GPIO_PORT,UART4_GPIO_PINSOURCE_TX,GPIO_AF_UART4);
		GPIO_PinAFConfig(UART4_GPIO_PORT,UART4_GPIO_PINSOURCE_RX,GPIO_AF_UART4);
	}
	
	serial_config(dev);
}




/* RT-Thread Device Interface */
static rt_err_t serial_init (rt_device_t dev)
{
    struct  serial_device* uart = (struct serial_device*) dev->user_data;

	/*if dev is not activated*/
    if (!(dev->flag & RT_DEVICE_FLAG_ACTIVATED))
    {
		/*initialize hardware*/
		uasrt_hw_init(uart);
		/*if dev is in int rx mode*/
        if (dev->flag & RT_DEVICE_FLAG_INT_RX)
        {
            rt_memset(uart->int_rx->rx_buffer, 0,
                sizeof(uart->int_rx->rx_buffer));
            uart->int_rx->read_index = 0;
            uart->int_rx->save_index = 0;
			/* enable interrupt */
			uasrt_nvic_configuration(uart->uart_device);
			USART_ITConfig(uart->uart_device, USART_IT_RXNE, ENABLE);
        }

        /* Enable USART */
        USART_Cmd(uart->uart_device, ENABLE);

        dev->flag |= RT_DEVICE_FLAG_ACTIVATED;
    }
    return RT_EOK;
}


static rt_err_t  serial_open(rt_device_t dev, uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t  serial_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t  serial_read (rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    rt_uint8_t* ptr;
    rt_err_t err_code;
    struct  serial_device* uart;

    ptr = buffer;
    err_code = RT_EOK;
    uart = (struct serial_device*)dev->user_data;

    if (dev->flag & RT_DEVICE_FLAG_INT_RX)
    {
        /* interrupt mode Rx */
        while (size)
        {
            rt_base_t level;

            /* disable interrupt */
            level = rt_hw_interrupt_disable();

            if (uart->int_rx->read_index != uart->int_rx->save_index)
            {
                /* read a character */
                *ptr++ = uart->int_rx->rx_buffer[uart->int_rx->read_index];
                size--; 

                /* move to next position */
                uart->int_rx->read_index ++;
                if (uart->int_rx->read_index >= UART_RX_BUFFER_SIZE)
                    uart->int_rx->read_index = 0;
            }
            else
            {
                /* set error code */
                err_code = -RT_EEMPTY;

                /* enable interrupt */
                rt_hw_interrupt_enable(level);
                break;
            }

            /* enable interrupt */
            rt_hw_interrupt_enable(level);
        }
    }
    else

    /* set error code */
    rt_set_errno(err_code);
    return (rt_uint32_t)ptr - (rt_uint32_t)buffer;
}

static rt_size_t  serial_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    rt_uint8_t* ptr;
    rt_err_t err_code;
    struct  serial_device* uart;

    err_code = RT_EOK;
    ptr = (rt_uint8_t*)buffer;
    uart = (struct serial_device*)dev->user_data;

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
        if (dev->flag & RT_DEVICE_FLAG_STREAM)
        {
            /* stream mode */
            while (size)
            {
                if (*ptr == '\n')
                {
                    while (!(uart->uart_device->SR & USART_FLAG_TXE));
                    uart->uart_device->DR = '\r';
                }

                while (!(uart->uart_device->SR & USART_FLAG_TXE));
                uart->uart_device->DR = (*ptr & 0x1FF);

                ++ptr; --size;
            }
        }
        else
        {
            /* write data directly */
            while (size)
            {
                while (!(uart->uart_device->SR & USART_FLAG_TXE));
                uart->uart_device->DR = (*ptr & 0x1FF);

                ++ptr; --size;
            }
        }
    }

    /* set error code */
    rt_set_errno(err_code);

    return (rt_uint32_t)ptr - (rt_uint32_t)buffer;
}



static rt_err_t  serial_control (rt_device_t dev, rt_uint8_t cmd, void *args)
{
    struct  serial_device* uart;

    RT_ASSERT(dev != RT_NULL);

    uart = (struct serial_device*)dev->user_data;
    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        /* suspend device */
        dev->flag |= RT_DEVICE_FLAG_SUSPENDED;
        USART_Cmd(uart->uart_device, DISABLE);
        break;

    case RT_DEVICE_CTRL_RESUME:
        /* resume device */
        dev->flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        USART_Cmd(uart->uart_device, ENABLE);
        break;
	case RT_DEVICE_CTRL_CONFIG:
		serial_config(uart);
	    break;
    }

    return RT_EOK;
}

void  hw_serial_isr(rt_device_t device)
{
    struct  serial_device* uart = (struct  serial_device*) device->user_data;

    if(USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        /* interrupt mode receive */
        RT_ASSERT(device->flag & RT_DEVICE_FLAG_INT_RX);

        /* save on rx buffer */
        while (uart->uart_device->SR & USART_FLAG_RXNE)
        {
            rt_base_t level;

            /* disable interrupt */
            level = rt_hw_interrupt_disable();

            /* save character */
            uart->int_rx->rx_buffer[uart->int_rx->save_index] = uart->uart_device->DR & 0xff;
            uart->int_rx->save_index ++;
            if (uart->int_rx->save_index >= UART_RX_BUFFER_SIZE)
                uart->int_rx->save_index = 0;

            /* if the next position is read index, discard this 'read char' */
            if (uart->int_rx->save_index == uart->int_rx->read_index)
            {
                uart->int_rx->read_index ++;
                if (uart->int_rx->read_index >= UART_RX_BUFFER_SIZE)
                    uart->int_rx->read_index = 0;
            }

            /* enable interrupt */
            rt_hw_interrupt_enable(level);
        }

        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_RXNE);

        /* invoke callback */
        if (device->rx_indicate != RT_NULL)
        {
            rt_size_t rx_length;

            /* get rx length */
            rx_length = uart->int_rx->read_index > uart->int_rx->save_index ?
                UART_RX_BUFFER_SIZE - uart->int_rx->read_index + uart->int_rx->save_index :
                uart->int_rx->save_index - uart->int_rx->read_index;

            device->rx_indicate(device, rx_length);
        }
    }

    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }
}

/*
 * serial register for STM32
 * support STM32F103VB and STM32F103ZE
 */
rt_err_t  serial_register(rt_device_t device, const char* name, rt_uint32_t flag, struct serial_device *serial)
{
    RT_ASSERT(device != RT_NULL);

    if ((flag & RT_DEVICE_FLAG_DMA_RX) ||
        (flag & RT_DEVICE_FLAG_INT_TX))
    {
        RT_ASSERT(0);
    }

    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->init        =  serial_init;
    device->open        =  serial_open;
    device->close       =  serial_close;
    device->read        =  serial_read;
    device->write       =  serial_write;
    device->control     =  serial_control;
    device->user_data   = serial;

    /* register a character device */
    return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR | flag);
}

void  hw_usart_init(void)
{
	serial_register(&usart1_device, "usart1",
        RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
        &usart1);
	serial_register(&usart2_device, "usart2",
        RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
        &usart2);
	serial_register(&usart3_device, "usart3",
        RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
        &usart3);
	serial_register(&uart4_device, "uart4",
        RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
        &uart4);
}

