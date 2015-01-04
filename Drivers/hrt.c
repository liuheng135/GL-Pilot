#include "hrt.h"

hrt_abstime Hrt_Time = 0;

void hrt_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = HRT_TIMER_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	RCC_APB1PeriphClockCmd(HRT_TIMER_CLK, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 50000;  
	TIM_TimeBaseStructure.TIM_Prescaler = 83;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(HRT_TIMER, &TIM_TimeBaseStructure);
	
	TIM_ClearFlag(HRT_TIMER,TIM_FLAG_Update | TIM_FLAG_CC1);
	
	TIM_ITConfig(HRT_TIMER,TIM_IT_Update|TIM_IT_CC1,ENABLE);
	
	TIM_Cmd(HRT_TIMER, ENABLE);
}

hrt_abstime hrt_absolute_time(void)
{
    return Hrt_Time + TIM4->CNT;
}

hrt_abstime hrt_absolute_ms(void)
{
    return (Hrt_Time + TIM4->CNT)/1000;
}

/**
 * Compare a time value with the current time.
 */
hrt_abstime hrt_elapsed_time(const volatile hrt_abstime then)
{
	hrt_abstime delta = hrt_absolute_time() - then;

	return delta;
}

