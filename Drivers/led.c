#include "stm32f4xx_conf.h"
#include "LED.h"

void LED_Init(Led_TypeDef Led)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	if(Led == LED1)
	{
	/* Enable the GPIO_LED Clock */
		RCC_AHB1PeriphClockCmd(LED1_CLK, ENABLE);

		/* Configure the GPIO_LED pin */
		GPIO_InitStructure.GPIO_Pin = LED1_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(LED1_PORT, &GPIO_InitStructure);
	}
	else if(Led == LED2)
	{
		RCC_AHB1PeriphClockCmd(LED2_CLK, ENABLE);

		/* Configure the GPIO_LED pin */
		GPIO_InitStructure.GPIO_Pin = LED2_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(LED2_PORT, &GPIO_InitStructure);
	}
	else if(Led == LED3)
	{
		RCC_AHB1PeriphClockCmd(LED3_CLK, ENABLE);

		/* Configure the GPIO_LED pin */
		GPIO_InitStructure.GPIO_Pin = LED3_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(LED3_PORT, &GPIO_InitStructure);
	}

}

void LED_ON(Led_TypeDef Led)
{
	if(Led == LED1)
	{
		GPIO_ResetBits(LED1_PORT,LED1_PIN);
	}
	else if(Led == LED2)
	{
		GPIO_ResetBits(LED2_PORT,LED2_PIN);
	}
	else if(Led == LED3)
	{
		GPIO_ResetBits(LED3_PORT,LED3_PIN);
	}
}

void LED_OFF(Led_TypeDef Led)
{
	if(Led == LED1)
	{
		GPIO_SetBits(LED1_PORT,LED1_PIN);
	}
	else if(Led == LED2)
	{
		GPIO_SetBits(LED2_PORT,LED2_PIN);
	}
	else if(Led == LED3)
	{
		GPIO_SetBits(LED3_PORT,LED3_PIN);
	}
}
