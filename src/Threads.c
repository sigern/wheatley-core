#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS:Keil RTX5
#include "stm32f4xx_hal.h"

#include "../include/LED.h"
#include "../include/Servo.h"
#include "../include/Bluetooth.h"
#include "../include/CRC.h"

extern void Error_Handler(void);

static osThreadId_t tid_thrLEDBlinker;           // Thread id of thread: LEDBlinker
static osThreadId_t tid_thrServoModulator;       // Thread id of thread: ServoModulator
static osThreadId_t tid_thrFrameParser;          // Thread id of thread: FrameParser
static osThreadId_t tid_thrBluetoothTransmitter; // Thread id of thread: Bluetooth Transmitter

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
	uint8_t msg;
	osStatus_t status;
  for (;;) {
		status = osMessageQueueGet(UartRxMsgQueueId, &msg, NULL, NULL);  // wait for message
		uint32_t input[] = {0x01020304, 0x05060708};
		uint32_t crc = 0;
		uint8_t textCrc[4] = {0};
		if (status == osOK) {
			crc = CRC_Calculate(&input, 2);
		   uint8_t *ptr = (uint8_t*)&crc;
			for(int i=0; i<4; ++i, ++ptr)
			{
				textCrc[i] = *ptr;
	  	}
			Bluetooth_Send(textCrc, 4);
		}
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
	
	/* Create serial transfer thread */
  tid_thrFrameParser = osThreadNew(thrFrameParser, NULL, NULL); 
  if (tid_thrFrameParser == NULL) { /* add error handling */ }
	
	/* Create serial transfer thread */
  //tid_thrBluetoothTransmitter = osThreadNew(thrBluetoothTransmitter, NULL, NULL); 
  //if (tid_thrServoModulator == NULL) { /* add error handling */ }

  osThreadExit();
}
