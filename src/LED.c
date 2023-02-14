#include "../include/LED.h"
#include "stm32f4xx_hal.h"

void LED_On(uint16_t ledColor) {
	HAL_GPIO_WritePin(GPIOD, ledColor, GPIO_PIN_SET);
}

void LED_Off(uint16_t ledColor) {
	HAL_GPIO_WritePin(GPIOD, ledColor, GPIO_PIN_RESET);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}