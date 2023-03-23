#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS:Keil RTX5
#include "stm32f4xx_hal.h"

#include "../include/LED.h"
#include "../include/Servo.h"
#include "../include/Bluetooth.h"
#include "../include/CRC.h"
#include "../include/defines.h"

extern void Error_Handler(void);

static osThreadId_t tid_thrLEDBlinker;           // Thread id of thread: LEDBlinker
static osThreadId_t tid_thrServoModulator;       // Thread id of thread: ServoModulator
static osThreadId_t tid_thrFrameParser;          // Thread id of thread: FrameParser
static osThreadId_t tid_thrSender;               // Thread id of thread: Sender
static osThreadId_t tid_thrController;           // Thread id of thread: Controller

extern UART_HandleTypeDef huart6;
extern uint8_t UART6_rxBuffer;
extern osMessageQueueId_t UartRxMsgQueueId;  // message queue id

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
  thrServoModulator: Modulate Servo
 *----------------------------------------------------------------------------*/
__NO_RETURN void thrServoModulator (void *argument) {
  for (;;) {
     //ServoTilt_Set(0);
    osDelay (2000U);  // Delay 2000 ms
     //ServoTilt_Set(19999);
    osDelay (2000U);  // Delay 2000 ms
  }
}

/*------------------------------------------------------------------------------
  thrFrameParser: Parse frame received via UART
 *----------------------------------------------------------------------------*/
__NO_RETURN void thrFrameParser (void *argument) {
	uint8_t receiverState = NONE;
	uint8_t msg = 0u;
	uint8_t tilt_cache = JOYSTICK_ZERO;
	uint8_t roll_cache = JOYSTICK_ZERO;
	osStatus_t status;
	
  for (;;) {
		status = osMessageQueueGet(UartRxMsgQueueId, &msg, NULL, NULL);  // wait for message
		if (status == osOK) {
      switch(receiverState) {
        case NONE:
          if (msg == FRAME_START) {
            receiverState = CHECK_TYPE;
          }
          break;
        case CHECK_TYPE:
          if (msg == FRAME_TYPE_JOYSTICK) {
						receiverState = JOYSTICK_TILT;
          } else {
            receiverState = NONE;
          }
          break;
        case JOYSTICK_TILT:
					tilt_cache = (uint8_t)msg;
          receiverState = JOYSTICK_ROLL;
          break;
				case JOYSTICK_ROLL:
          roll_cache = (uint8_t)msg;
          receiverState = CHECK_END;
          break;
				case CHECK_END:
					if (msg == FRAME_END) {
						g_joystick.tilt = tilt_cache;
						g_joystick.roll = roll_cache;
          }
          receiverState = NONE;
          break;
        default:
          receiverState = NONE;
          break;
      }
		}
		
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
		uint32_t queueSize = osMessageQueueGetCount(UartRxMsgQueueId);
		osDelay (100);  // Delay 2000 ms
		Bluetooth_Send(testFrame, sizeof(testFrame));
		testTilt++;
		testRoll++;
		testFrame[2] = (uint8_t)(queueSize >> 8 & 0xFF);
		testFrame[3] = (uint8_t)(queueSize & 0xFF);
		testFrame[4] = (uint8_t)(0 >> 8 & 0xFF);
		testFrame[5] = (uint8_t)(g_joystick.roll & 0xFF);
  }
}

/*------------------------------------------------------------------------------
 * Application main thread
 *----------------------------------------------------------------------------*/
void app_main (void *argument) {

	 /* Initialize Board LEDs */
	 LED_Init();
	 /* Initialize PWM for roll and tilt servos */
	 Servo_Init();
	 /* Initialize USART and DMA for serial communication via Bluetooth */
	 Bluetooth_Init();
	 /* Initialize CRC peripheral for communication frame checksum calculation */
	 CRC_Init();


	/* Create LED blink thread */
  tid_thrLEDBlinker = osThreadNew(thrLEDBlinker, NULL, NULL); 
  if (tid_thrLEDBlinker == NULL) { /* add error handling */ }
	
	/* Create Servo Modulator thread */
  tid_thrServoModulator = osThreadNew(thrServoModulator, NULL, NULL); 
  if (tid_thrServoModulator == NULL) { /* add error handling */ }
	
	/* Create RX frame parser thread */
  tid_thrFrameParser = osThreadNew(thrFrameParser, NULL, NULL); 
  if (tid_thrFrameParser == NULL) { /* add error handling */ }
	
	/* Create TX frame sender thread */
  tid_thrSender = osThreadNew(thrSender, NULL, NULL); 
  if (tid_thrSender == NULL) { /* add error handling */ }

  osThreadExit();
}
