#ifndef __ACCELEROMETER_H
#define __ACCELEROMETER_H

#include "stm32f4xx_hal.h"

typedef enum 
{
  ACCELERO_OK = 0,
  ACCELERO_ERROR = 1,
  ACCELERO_TIMEOUT = 2
} ACCELERO_StatusTypeDef;

uint8_t ACC_Init(void);
void    ACC_Reset(void);
void    BSP_ACCELERO_Click_ITConfig(void);
void    ACC_GetXYZ(int16_t *pDataXYZ);

#define ABS(x)         (x < 0) ? (-x) : x

void ACCELERO_MEMS_Test(void);

#endif /* __ACCELEROMETER_H */
