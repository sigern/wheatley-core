#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS:Keil RTX5
#include "stm32f4xx_hal.h"

#include "../include/LED.h"
#include "../include/Servo.h"
#include "../include/Bluetooth.h"

extern void Error_Handler(void);

static osThreadId_t tid_thrLEDBlinker;           // Thread id of thread: LEDBlinker
static osThreadId_t tid_thrServoModulator;       // Thread id of thread: ServoModulator
static osThreadId_t tid_thrBluetoothTransmitter; // Thread id of thread: Bluetooth Transmitter

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern uint8_t UART6_rxBuffer[12];

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
  thrLEDBlinker: Modulate Servo
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
  thrLEDBlinker: Transmit data via Bluetooth
 *----------------------------------------------------------------------------*/
__NO_RETURN void thrBluetoothTransmitter (void *argument) {
  //if(HAL_UART_Receive_DMA(&huart6, (uint8_t *)UART6_rxBuffer, 12) != HAL_OK)
 // {
 //   Error_Handler();
 // }
  for (;;) {
		//Bluetooth_Send(UART6_rxBuffer);
    osDelay (1000U);  // Delay 2000 ms
  }
}

/*------------------------------------------------------------------------------
 * Application main thread
 *----------------------------------------------------------------------------*/
void app_main (void *argument) {

	 /* Initialize Board LEDs */
	 LED_Init();
	 /* Initialize PWM for roll and tilt servos */
	 ServosInit();
		/* Initialize USART and DMA for serial communication via Bluetooth */
	 Bluetooth_Init();

	/* Create LED blink thread */
  tid_thrLEDBlinker = osThreadNew(thrLEDBlinker, NULL, NULL); 
  if (tid_thrLEDBlinker == NULL) { /* add error handling */ }
	
	/* Create Servo Modulator thread */
  tid_thrServoModulator = osThreadNew(thrServoModulator, NULL, NULL); 
  if (tid_thrServoModulator == NULL) { /* add error handling */ }
	
	/* Create serial transfer thread */
  //tid_thrBluetoothTransmitter = osThreadNew(thrBluetoothTransmitter, NULL, NULL); 
  //if (tid_thrServoModulator == NULL) { /* add error handling */ }

  osThreadExit();
}
