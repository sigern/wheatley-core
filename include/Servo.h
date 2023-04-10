#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define ROLL_FORWARD_LIMIT    36935 // ROLL_ZERO - 165
#define ROLL_ZERO             37110
#define ROLL_BACK_LIMIT       37275 // ROLL_ZERO + 165
#define ROLL_DELTA            50

#define TILT_LEFT_LIMIT       6590 // TILT_ZERO - 4040
#define TILT_ZERO             10630
#define TILT_RIGHT_LIMIT      14670 // TILT_ZERO + 4040
#define TILT_DELTA            4040

void Servo_Init(void);
void Servo_Enable(bool isEnable);
void ServoRoll_Set(uint16_t value);
void ServoTilt_Set(uint16_t value);

#endif /* __SERVO_H */
