#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "cmsis_os2.h"      

void Bluetooth_Init();
void Bluetooth_Send(uint8_t* data, uint8_t length);

#define UART_RX_QUEUE_SIZE 16

#endif /* __BLUETOOTH_H */

