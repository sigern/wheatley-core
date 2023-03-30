 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEFINES_H
#define __DEFINES_H

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "../include/Servo.h"

#define JOYSTICK_MIN  0
#define JOYSTICK_ZERO 120
#define JOYSTICK_MAX  240

enum EFrame {
    FRAME_START = 0xF0,
    FRAME_END   = 0xF1,
    FRAME_TYPE_JOYSTICK = 0xF2,
    FRAME_TYPE_SERVO = 0xF3,
    FRAME_TYPE_LIPOL = 0xF4,
    FRAME_TYPE_VELOCITY = 0xF5,
    FRAME_TYPE_SERVO_ENABLED = 0xF6,
	  FRAME_TYPE_HEARTBEAT = 0xF7
};

enum EReceiverState {
    NONE,
    CHECK_TYPE,
	  CHECK_END,
    JOYSTICK_TILT,
	  JOYSTICK_ROLL,
	  JOYSTICK_CRC,
	  SERVO_ENABLED
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

// Global variables
volatile RobotState_t g_wheatley = {TILT_ZERO, ROLL_ZERO, 0.f, 0.f, 0.f, 0.f, 0.f};
volatile JoystickState_t g_joystick = {JOYSTICK_ZERO, JOYSTICK_ZERO};

volatile uint32_t g_heartbeat_timestamp_ms = 0u;

// Mutexes for global variables
osMutexId_t joystickStateMutex_id;  
osMutexId_t robotStateMutex_id;  
 
const osMutexAttr_t joystickStateMutex_attr = {
  "joystickStateMutex",// human readable mutex name
  osMutexPrioInherit,  // attr_bits
  NULL,                // memory for control block   
  0U                   // size for control block
};

const osMutexAttr_t robotStateMutex_attr = {
  "robotStateMutex",// human readable mutex name
  osMutexPrioInherit,  // attr_bits
  NULL,                // memory for control block   
  0U                   // size for control block
};


#endif /* __DEFINES_H */

