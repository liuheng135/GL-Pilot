#ifndef __HRT_H_
#define __HRT_H

#include "stm32f4xx.h"

extern uint64_t Hrt_Time;
typedef uint64_t hrt_abstime;

#define USE_HRT_TIMER   4

#if   USE_HRT_TIMER == 4

#define HRT_TIMER      TIM4
#define HRT_TIMER_CLK  RCC_APB1Periph_TIM4
#define HRT_TIMER_IRQ  TIM4_IRQn

#endif

void hrt_init(void);
hrt_abstime hrt_absolute_time(void);
hrt_abstime hrt_absolute_ms(void);



#endif
