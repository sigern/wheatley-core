#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"

#define ROLL_BACK_LIMIT       36940 // ROLL_ZERO - 165
#define ROLL_BACK_DEADZONE    37085 // ROLL_ZERO - 20
#define ROLL_ZERO             37105
#define ROLL_FORWARD_DEADZONE 37125 // ROLL_ZERO + 20
#define ROLL_FORWARD_LIMIT    37270 // ROLL_ZERO + 165

void Servo_Init(void);

#endif /* __SERVO_H */
