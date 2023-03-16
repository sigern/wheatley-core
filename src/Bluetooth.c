#include "../include/Bluetooth.h"

UART_HandleTypeDef huart6;

osMessageQueueId_t UartRxMsgQueueId;  // message queue id
uint8_t UART6_rxBuffer;

extern void Error_Handler(void);

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance == USART6)
  {
		/* Enable GPIO TX/RX clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();
		/* Enable USART6 clock */
    __HAL_RCC_USART6_CLK_ENABLE();

    /* Configure peripheral GPIO for USART6 */
    /* PC6 ------> USART6_TX */
    /* PC7 ------> USART6_RX */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
		
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		/* Configure NVIC for USART TC interrupt */
		HAL_NVIC_SetPriority(USART6_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(USART6_IRQn);
  }
}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance == USART6)
  {
    /* Disable peripheral clock  */
    __HAL_RCC_USART6_CLK_DISABLE();
    /* Disable peripherals and GPIO Clocks */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);
  }
}

/**
  * @brief  This function handles USART6 interrupt request.
  */
void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart6);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART6)
	{
		osMessageQueuePut (UartRxMsgQueueId, &UART6_rxBuffer, 0, NULL);		
		if(HAL_UART_Receive_IT(&huart6, &UART6_rxBuffer, 1) != HAL_OK)
		{
			Error_Handler();
		}
	}
}

/**
  * @brief USART6 Initialization Function
  */
void Bluetooth_Init() 
{
	/* USART6 initialization */
	huart6.Instance = USART6;
  huart6.Init.BaudRate     = 115200;
  huart6.Init.WordLength   = UART_WORDLENGTH_8B;
  huart6.Init.StopBits     = UART_STOPBITS_1;
  huart6.Init.Parity       = UART_PARITY_NONE;
  huart6.Init.Mode         = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_8;
	
  if (HAL_UART_Init(&huart6) != HAL_OK) {
    Error_Handler();
  }
	
	if(HAL_UART_Receive_IT(&huart6, &UART6_rxBuffer, 1) != HAL_OK) {
		Error_Handler();
  }
	
	UartRxMsgQueueId = osMessageQueueNew(UART_RX_QUEUE_SIZE, sizeof(uint8_t), NULL);
  if (!UartRxMsgQueueId) {
    Error_Handler();
  }
}

void Bluetooth_Send(uint8_t* data, uint8_t length)
{
	HAL_UART_Transmit(&huart6, data, length, 0xFF);
}