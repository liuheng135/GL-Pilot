#include "app_led.h"



void app_led_main(void* parameter)
{
	LED_Init(LED1);
	LED_Init(LED2);
	LED_Init(LED3);
	
	while(1)
	{
		LED_ON(LED1);
		rt_thread_delay(500);
		LED_OFF(LED1);
		rt_thread_delay(500);	
	}
}
