#ifndef __CRC_H
#define __CRC_H

#include "stm32f4xx_hal.h"

uint32_t CRC_Calculate(uint32_t* buffer, uint32_t length);
void CRC_Init(void);

#endif /* __CRC_H */
