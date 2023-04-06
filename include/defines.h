 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEFINES_H
#define __DEFINES_H

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "../include/Servo.h"

#define JOYSTICK_MIN  0
#define JOYSTICK_ZERO 120
#define JOYSTICK_MAX  240

/* Complementary filter defines */
#define CF_TIMESTEP_SEC 0.02f
#define CF_EPS          0.08f

/* Setpoint conversion defines */
#define SP_P1 0.00000538f
#define SP_P2 0.00002117f
#define SP_P3 0.01741717f

/* PID defines */
#define C_KP   0.7f//1.0f //0.751
#define C_KI   -0.2f//-0.8f //-0.5f
#define C_KD   -0.2f//-0.11f//-0.125f
#define	C_TD   0.1f//0.1f
#define C_TP   0.014f
#define C_TRES 0.0f
#define C_TF   0.14f

enum EFrame {
	FRAME_START = 0xF0,
	FRAME_END   = 0xF1,
	FRAME_TYPE_JOYSTICK = 0xF2,
	FRAME_TYPE_ROBOT_STATE = 0xF3,
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
	int16_t tilt_angle;
	int16_t roll_angle;
	int16_t velocity;
	uint16_t lipol_vol;
} RobotState_t;

// Global variables
volatile RobotState_t g_wheatley = {TILT_ZERO, ROLL_ZERO, 0, 0, 0, 0u};
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
  "robotStateMutex",  // human readable mutex name
  osMutexPrioInherit, // attr_bits
  NULL,               // memory for control block   
  0U                  // size for control block
};

const osMutexAttr_t sensorStateMutex_attr = {
  "sensorStateMutex", // human readable mutex name
  osMutexPrioInherit, // attr_bits
  NULL,               // memory for control block   
  0U                  // size for control block
};



#endif /* __DEFINES_H */

