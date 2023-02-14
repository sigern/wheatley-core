#ifndef __LED_H
#define __LED_H

#include "stm32f4xx_hal.h"

typedef enum {
	GREEN  = GPIO_PIN_12,
	ORANGE = GPIO_PIN_13,
	RED    = GPIO_PIN_14,
	BLUE   = GPIO_PIN_15
} ELEDColor;

void LED_On(uint16_t ledColor);
void LED_Off(uint16_t ledColor);
void LED_Init(void);

#endif /* __LED_H */
