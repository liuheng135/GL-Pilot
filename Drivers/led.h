#ifndef __LED_H__
#define __LED_H__

typedef enum {
	LED1 = 0,
	LED2,
	LED3,
}Led_TypeDef;


#define LED1_PIN     GPIO_Pin_3
#define LED1_PORT    GPIOB
#define LED1_CLK     RCC_AHB1Periph_GPIOB

#define LED2_PIN     GPIO_Pin_4
#define LED2_PORT    GPIOB
#define LED2_CLK     RCC_AHB1Periph_GPIOB

#define LED3_PIN     GPIO_Pin_5
#define LED3_PORT    GPIOB
#define LED3_CLK     RCC_AHB1Periph_GPIOB


void LED_Init(Led_TypeDef Led);
void LED_ON(Led_TypeDef Led);
void LED_OFF(Led_TypeDef Led);

#endif
