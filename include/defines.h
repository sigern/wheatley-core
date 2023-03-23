 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEFINES_H
#define __DEFINES_H

#include "stm32f4xx_hal.h"

#define JOYSTICK_MIN  0
#define JOYSTICK_ZERO 120
#define JOYSTICK_MAX  240

enum EFrame {
    FRAME_START = 0xF0,
    FRAME_END   = 0xF1,
    FRAME_TYPE_JOYSTICK = 0xF2,
    FRAME_TYPE_SERVO = 0xF3,
    FRAME_TYPE_LIPOL = 0xF4,
    FRAME_TYPE_VELOCITY = 0xF5
};

enum EReceiverState {
    NONE,
    CHECK_TYPE,
	  CHECK_END,
    JOYSTICK_TILT,
	  JOYSTICK_ROLL
};

typedef struct joystickState
{
    uint8_t tilt;
    uint8_t roll;
} JoystickState_t;

typedef struct robotState
{
    uint16_t tilt_servo;
    uint16_t roll_servo;
    float velocity;
    float lipol_vol;
    float gain_p;
    float gain_i;
    float gain_d;
} RobotState_t;


volatile RobotState_t g_wheatley = {0u, 0u, 0.f, 0.f, 0.f, 0.f, 0.f};
volatile JoystickState_t g_joystick = {JOYSTICK_ZERO, JOYSTICK_ZERO};

#endif /* __DEFINES_H */

