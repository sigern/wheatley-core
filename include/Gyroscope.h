#ifndef __GYROSCOPE_H
#define __GYROSCOPE_H

#include "stm32f4xx_hal.h"

typedef enum 
{
  GYRO_OK = 0,
  GYRO_ERROR = 1,
  GYRO_TIMEOUT = 2
} GYRO_StatusTypeDef;

/* Gyroscope Functions */ 
uint8_t GYRO_Init(void);
void    GYRO_Reset(void);
uint8_t GYRO_ReadID(void);
void    GYRO_EnableIT(uint8_t IntPin);
void    GYRO_DisableIT(uint8_t IntPin);
void    GYRO_GetXYZ(int16_t *pfData);

#endif /* __GYROSCOPE_H */
