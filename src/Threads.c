#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS:Keil RTX5
#include "stm32f4xx_hal.h"
#include "arm_math.h"  

#include "../include/LED.h"
#include "../include/Accelerometer.h"
#include "../include/Gyroscope.h"
#include "../include/Servo.h"
#include "../include/Bluetooth.h"
#include "../include/CRC.h"
#include "../include/defines.h"

static osThreadId_t tid_thrLEDBlinker;           // Thread id of thread: LEDBlinker
static osThreadId_t tid_thrFrameParser;          // Thread id of thread: FrameParser
static osThreadId_t tid_thrSender;               // Thread id of thread: Sender
static osThreadId_t tid_thrController;           // Thread id of thread: Controller

extern UART_HandleTypeDef huart6;
extern uint8_t UART6_rxBuffer;
extern osMessageQueueId_t UartRxMsgQueueId;  // message queue id

float32_t g_tiltAngle = 0.f;
float32_t g_rollAngle = 0.f;
float32_t g_gyroX = 0.f;
float32_t g_gyroY = 0.f;
float32_t g_gyroZ = 0.f;
int16_t g_accX = 0.f;
int16_t g_accY = 0.f;
int16_t g_accZ = 0.f;


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(uint8_t val)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	if (val == 0) {
		LED_On(GREEN);
	} else if (val == 1) {
		LED_On(ORANGE);
	} else if (val == 2) {
	  LED_On(RED);
	} else {
		LED_On(BLUE);
	}
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/*------------------------------------------------------------------------------
  thrLEDBlinker: blink LED
 *----------------------------------------------------------------------------*/
__NO_RETURN void thrLEDBlinker (void *argument) {
  for (;;) {
    LED_On(RED | ORANGE | BLUE | GREEN);
    osDelay (500U);  // Delay 500 ms
    LED_Off(RED | ORANGE | BLUE | GREEN);	
    osDelay (500U);  // Delay 500 ms
  }
}

/*------------------------------------------------------------------------------
  thrFrameParser: Parse frame received via UART
 *----------------------------------------------------------------------------*/
__NO_RETURN void thrFrameParser (void *argument) {
	uint8_t sm_state = NONE;
  uint8_t	current_frame = NONE;
	uint8_t msg = 0u;
	
	bool servo_enabled_cache = false;
	uint8_t tilt_cache = JOYSTICK_ZERO;
	uint8_t roll_cache = JOYSTICK_ZERO;
	uint8_t crc_cache = 0u;
	
	osStatus_t status;
	
  for (;;) {
		status = osMessageQueueGet(UartRxMsgQueueId, &msg, NULL, 100);  // wait for message
		if (status == osOK) {
      switch(sm_state) {
        case NONE:
          if (msg == FRAME_START) {
            sm_state = CHECK_TYPE;
          }
          break;
        case CHECK_TYPE:
          if (msg == FRAME_TYPE_JOYSTICK) {
						sm_state = JOYSTICK_TILT;
						current_frame = FRAME_TYPE_JOYSTICK;
					} else if (msg == FRAME_TYPE_SERVO_ENABLED) {
						sm_state = SERVO_ENABLED;
						current_frame = FRAME_TYPE_SERVO_ENABLED;
          }	else if (msg == FRAME_TYPE_HEARTBEAT) {
						sm_state = CHECK_END;
						current_frame = FRAME_TYPE_HEARTBEAT;
          }else {
            sm_state = NONE;
						current_frame = NONE;
          }
          break;
        case JOYSTICK_TILT:
					tilt_cache = (uint8_t)msg;
          sm_state = JOYSTICK_ROLL;
          break;
				case JOYSTICK_ROLL:
          roll_cache = (uint8_t)msg;
          sm_state = JOYSTICK_CRC;
          break;
				case JOYSTICK_CRC:
          crc_cache = (uint8_t)msg;
					uint8_t crc_input[] = {tilt_cache, roll_cache};
					if (CRC8(crc_input, 2) == crc_cache) {
						sm_state = CHECK_END;
					} else {
				    sm_state = NONE;
						current_frame = NONE;
					}
          break;
				case SERVO_ENABLED:
					servo_enabled_cache = (uint8_t)msg;
				  sm_state = CHECK_END;
				  break;
				case CHECK_END:
					if (msg == FRAME_END) {
						if (current_frame == FRAME_TYPE_JOYSTICK) {
							osMutexAcquire(joystickStateMutex_id, osWaitForever);
							g_joystick.tilt = tilt_cache;
						  g_joystick.roll = roll_cache;
							osMutexRelease(joystickStateMutex_id);
						} else if (current_frame == FRAME_TYPE_SERVO_ENABLED) {
							//Servo_Enable(servo_enabled_cache);
							tilt_pid_enable = (bool)servo_enabled_cache;
						}	else if (current_frame == FRAME_TYPE_HEARTBEAT) {
							g_heartbeat_timestamp_ms = HAL_GetTick();
						}								
          }
          sm_state = NONE;
					current_frame = NONE;
          break;
        default:
          sm_state = NONE;
				  current_frame = NONE;
          break;
      }
		}
	}
}

/*------------------------------------------------------------------------------
  thrSender: Send frame via UART
 *----------------------------------------------------------------------------*/
__NO_RETURN void thrSender (void *argument) {
	uint8_t robotStateFrame[15];
	robotStateFrame[0] = FRAME_START;
	robotStateFrame[1] = FRAME_TYPE_ROBOT_STATE;
	robotStateFrame[14] = FRAME_END;
  for (;;) {
		osDelay (150);
		memcpy(&robotStateFrame[2], (uint8_t*)&g_wheatley, 12);
		Bluetooth_Send(robotStateFrame, sizeof(robotStateFrame));

  }
}

float32_t calcComplementaryFilter(float32_t prev_angle, int16_t acc_1, int16_t acc_2, float gyro)
{
	float32_t atan = 0.f;
	float32_t acc_tilt_angle = 0.f;
	
	arm_atan2_f32((float32_t)acc_1, (float32_t)acc_2, &atan);
	acc_tilt_angle = 180.f * atan / PI;
	
	return (1.0 - CF_EPS)*(prev_angle - CF_TIMESTEP_SEC * gyro) + CF_EPS * acc_tilt_angle;
}

/*------------------------------------------------------------------------------
  thrSender: Control algorithm loop
 *----------------------------------------------------------------------------*/
__NO_RETURN void thrController (void *argument) {
	static const float32_t s_E1 = (KP*TD) + KD;
  static const float32_t s_E2 = (KI*TPT*TD) + KP*(TPT - 2*TD) - 2*KD;
  static const float32_t s_E3 = (TPT-TD)*(KI*TPT - KP) + KD;
  static const float32_t s_U1 = 2*TD - TPT;
  static const float32_t s_U2 = TPT - TD;
	
	int16_t accData[3];
	float gyroData[3];
	
	uint16_t sp_roll_joystick = JOYSTICK_ZERO;
	uint16_t sp_tilt_joystick = JOYSTICK_ZERO;
	float32_t sp_tilt_servo_angle = 0.f;
	float32_t sp_tilt_sphere_angle = 0.f;
	float32_t sp_tilt_servo_angle_inertial = 0.f;
	
	float32_t prev_tilt_sphere_angle = 0.f;
	float32_t prev_roll_sphere_angle = 0.f;
	float32_t roll_sphere_angle = 0.f;
	float32_t tilt_sphere_angle = 0.f;
	
	float32_t error_tilt = 0.f;
	float32_t prev_error_tilt = 0.f;
	float32_t prev2_error_tilt = 0.f;

	float32_t control_end_roll = JOYSTICK_ZERO;
	float32_t control_end_tilt = 0.f;
	float32_t control_tilt = 0.f;
	float32_t prev_control_tilt = 0.f;
	float32_t prev2_control_tilt = 0.f;

	uint16_t output_roll = ROLL_ZERO;
	uint16_t output_tilt = TILT_ZERO;
	
  for (;;) {
		osDelay (20);
		
		/* Get ACC and GYRO sensors data */
		ACC_GetXYZ(accData);
		GYRO_GetXYZ(gyroData);
		
		/* Calculate complementary filter for tilt and row angles */
	  tilt_sphere_angle = calcComplementaryFilter(prev_tilt_sphere_angle, accData[0], accData[2], gyroData[0]);
		g_wheatley.tilt_angle = (int16_t)(tilt_sphere_angle * 10);
		prev_tilt_sphere_angle = tilt_sphere_angle;
		
		roll_sphere_angle = calcComplementaryFilter(prev_roll_sphere_angle, accData[1], accData[2], gyroData[1]);
		g_wheatley.roll_angle = (int16_t)(roll_sphere_angle * 10);
		prev_roll_sphere_angle = roll_sphere_angle;
		
		/* Check if last heartbeat is less than 2000 ms old, otherwise reset servos setpoint */
		if (HAL_GetTick() - g_heartbeat_timestamp_ms < 2000u) {
			osMutexAcquire(joystickStateMutex_id, osWaitForever);
			sp_roll_joystick = g_joystick.roll;
		  sp_tilt_joystick = g_joystick.tilt;
		  osMutexRelease(joystickStateMutex_id);
			
			/* Convert joystick tilt setpoint (0-240) to servo tilt angle setpoint (-60 to 60) */
			sp_tilt_servo_angle = (float32_t)(sp_tilt_joystick - JOYSTICK_ZERO) / 2.f;
			sp_tilt_servo_angle_inertial = sp_tilt_servo_angle_inertial*TF_T/(TF_T+TPR_T) + sp_tilt_servo_angle*TPR_T/(TF_T+TPR_T);
			sp_tilt_sphere_angle = 
			P1 * sp_tilt_servo_angle_inertial * sp_tilt_servo_angle_inertial * sp_tilt_servo_angle_inertial + 
			P2 * sp_tilt_servo_angle_inertial * sp_tilt_servo_angle_inertial + 
			P3 * sp_tilt_servo_angle_inertial;
			
			/* PID algorithm for tilt */
			error_tilt = sp_tilt_sphere_angle - tilt_sphere_angle;
			control_tilt = 
				(error_tilt * s_E1 + 
				prev_error_tilt * s_E2 +
				prev2_error_tilt * s_E3 + 
				prev_control_tilt * s_U1 + 
				prev2_control_tilt * s_U2) / TD;
			
			/* Control saturation */
			if (control_tilt > 100.f) {
				control_tilt = 100.f;
			} else if (control_tilt < -100.f) {
				control_tilt = -100.f;
			}
			
			/* Previous values */
			prev2_control_tilt = prev_control_tilt;
			prev_control_tilt = control_tilt;
			prev2_error_tilt = prev_error_tilt;
			prev_error_tilt = error_tilt;
			
			control_end_tilt = tilt_pid_enable ? sp_tilt_servo_angle_inertial - control_tilt : sp_tilt_servo_angle_inertial;
			output_tilt = TILT_ZERO + (int16_t)((control_end_tilt*2.f / (float)JOYSTICK_ZERO) * TILT_DELTA);
			
			/* Control end saturation */
			if (control_end_tilt > 60.f) {
				control_end_tilt = 60.f;
			} else if (control_end_tilt < -60.f) {
				control_end_tilt = -60.f;
			}
		
			/* Roll has no PID, only inertion */
			control_end_roll = control_end_roll*TF/(TF+TPR) + sp_roll_joystick*TPR/(TF+TPR);
			output_roll = ROLL_ZERO + (int16_t)((((float)control_end_roll - (float)JOYSTICK_ZERO) / (float)JOYSTICK_ZERO) * ROLL_DELTA);
			
		} else {
			/* Reset all servo input */
			sp_roll_joystick = JOYSTICK_ZERO;
			sp_tilt_joystick = JOYSTICK_ZERO;
			output_roll = ROLL_ZERO;
			output_tilt = TILT_ZERO;
		}
		
		/* fill global struct */
		osMutexAcquire(robotStateMutex_id, osWaitForever);
		g_wheatley.roll_servo = output_roll;
		g_wheatley.tilt_servo = output_tilt;
		osMutexRelease(robotStateMutex_id);

		if (output_roll >= ROLL_FORWARD_LIMIT && output_roll <= ROLL_BACK_LIMIT) {
			ServoRoll_Set(output_roll);
		}
		if (output_tilt <= TILT_RIGHT_LIMIT && output_tilt >= TILT_LEFT_LIMIT) {
			ServoTilt_Set(output_tilt);
		}
  }
}

void Create_Mutexes() {
	joystickStateMutex_id = osMutexNew(&joystickStateMutex_attr);
	robotStateMutex_id = osMutexNew(&robotStateMutex_attr);
}

/*------------------------------------------------------------------------------
 * Application main thread
 *----------------------------------------------------------------------------*/
void app_main (void *argument) {

	 /* Initialize Board LEDs */
	 LED_Init();
	 /* Initialize LSM303DLHC accelerometer */
	 ACC_Init();
   /* Initialize LSM303DLHC accelerometer */
	 GYRO_Init();
	 /* Initialize PWM for roll and tilt servos */
	 Servo_Init();
	 /* Initialize USART and DMA for serial communication via Bluetooth */
	 Bluetooth_Init();
	
	 /* Create mutexes for global variables */
	 Create_Mutexes();

	/* Create LED blink thread */
  tid_thrLEDBlinker = osThreadNew(thrLEDBlinker, NULL, NULL); 
  if (tid_thrLEDBlinker == NULL) { /* add error handling */ }
	
	/* Create RX frame parser thread */
  tid_thrFrameParser = osThreadNew(thrFrameParser, NULL, NULL); 
  if (tid_thrFrameParser == NULL) { /* add error handling */ }
	
	/* Create TX frame sender thread */
  tid_thrSender = osThreadNew(thrSender, NULL, NULL); 
  if (tid_thrSender == NULL) { /* add error handling */ }
	
	/* Create servo controller thread */
  tid_thrController = osThreadNew(thrController, NULL, NULL); 
  if (tid_thrController == NULL) { /* add error handling */ }

  osThreadExit();
}
