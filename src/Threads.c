#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS:Keil RTX5
#include "stm32f4xx_hal.h"

#include "../include/LED.h"
#include "../include/Accelerometer.h"
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
	
	bool new_joystick_input = false;
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
							g_joystick.tilt = tilt_cache;
						  g_joystick.roll = roll_cache;
							new_joystick_input = true;
						} else if (current_frame == FRAME_TYPE_SERVO_ENABLED) {
							Servo_Enable(servo_enabled_cache);
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
		#if 0
		if(g_joystick.tilt < 60) {
			LED_Off(RED);
			LED_On(GREEN);
    } else if (g_joystick.tilt > 180) {
			LED_Off(GREEN);
			LED_On(RED);
		} else {
			LED_Off(RED);
			LED_Off(GREEN);
		}
		

		if(g_joystick.roll < 60) {
			LED_Off(ORANGE);
			LED_On(BLUE);
    } else if (g_joystick.roll > 180) {
			LED_Off(BLUE);
			LED_On(ORANGE);
		} else {
			LED_Off(BLUE);
			LED_Off(ORANGE);
		}
		#endif
		if (new_joystick_input) {
			new_joystick_input = false;
		  osMutexAcquire(robotStateMutex_id, osWaitForever);
		  g_wheatley.roll_servo = ROLL_BACK_LIMIT + (uint16_t)((float)g_joystick.roll / (float)JOYSTICK_ZERO * (float)ROLL_DELTA);
		  g_wheatley.tilt_servo = TILT_LEFT_LIMIT + (uint16_t)((float)g_joystick.tilt / (float)JOYSTICK_ZERO * (float)TILT_DELTA);
		  osMutexRelease(robotStateMutex_id);
		}
	}
}

/*------------------------------------------------------------------------------
  thrSender: Send frame via UART
 *----------------------------------------------------------------------------*/
__NO_RETURN void thrSender (void *argument) {
	uint16_t testTilt = 0;
	uint16_t testRoll = 0;
	uint8_t testFrame[] = {
    FRAME_START, 
	  FRAME_TYPE_SERVO, 
	  (uint8_t)(testTilt & 0xFF),
	  (uint8_t)(testTilt >> 8 & 0xFF), 
	  (uint8_t)(testRoll & 0xFF), 
	  (uint8_t)(testRoll >> 8	& 0xFF),
    FRAME_END
	};
  for (;;) {
		osDelay (250);
		int16_t accData[3];
		ACC_GetXYZ(accData);
		Bluetooth_Send(testFrame, sizeof(testFrame));
		testFrame[2] = (uint8_t)(accData[0] >> 8 & 0xFF);
		testFrame[3] = (uint8_t)(accData[0] & 0xFF);
		testFrame[4] = (uint8_t)(g_wheatley.roll_servo >> 8 & 0xFF);
		testFrame[5] = (uint8_t)(g_wheatley.roll_servo & 0xFF);
  }
}

/*------------------------------------------------------------------------------
  thrSender: Send frame via UART
 *----------------------------------------------------------------------------*/
__NO_RETURN void thrController (void *argument) {
	uint16_t roll = ROLL_ZERO;
	uint16_t tilt = TILT_ZERO;
  for (;;) {
		osDelay (20);
		
		/* Check if last heartbeat is less than 2000 ms old, otherwise reset servos */
		if (HAL_GetTick() - g_heartbeat_timestamp_ms < 2000u) {
			osMutexAcquire(robotStateMutex_id, osWaitForever);
			roll = g_wheatley.roll_servo;
		  tilt = g_wheatley.tilt_servo;
		  osMutexRelease(robotStateMutex_id);
		} else {
			roll = ROLL_ZERO;
			tilt = TILT_ZERO;
		}

		if (roll >= ROLL_BACK_LIMIT && roll <= ROLL_FORWARD_LIMIT) {
			ServoRoll_Set(roll);
		}
		if (tilt <= TILT_RIGHT_LIMIT && tilt >= TILT_LEFT_LIMIT) {
			ServoTilt_Set(tilt);
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
	 /* Initialize PWM for roll and tilt servos */
	 //Servo_Init();
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
