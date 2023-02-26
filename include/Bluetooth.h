#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

void Bluetooth_Init();
void Bluetooth_Send(uint8_t* data);


#endif /* __BLUETOOTH_H */

