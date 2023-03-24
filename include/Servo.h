#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define ROLL_BACK_LIMIT       36940 // ROLL_ZERO - 165
#define ROLL_BACK_DEADZONE    37085 // ROLL_ZERO - 20
#define ROLL_ZERO             37105
#define ROLL_FORWARD_DEADZONE 37125 // ROLL_ZERO + 20
#define ROLL_FORWARD_LIMIT    37270 // ROLL_ZERO + 165
#define ROLL_DELTA            165

#define TILT_LEFT_LIMIT       6590 // TILT_ZERO - 80
#define TILT_ZERO             10630
#define TILT_RIGHT_LIMIT      14670 // TILT_ZERO + 80
#define TILT_DELTA            4040

void Servo_Init(void);
void Servo_Enable(bool isEnable);
void ServoRoll_Set(uint16_t value);
void ServoTilt_Set(uint16_t value);

#endif /* __SERVO_H */
