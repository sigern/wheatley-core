#include "../include/Bluetooth.h"

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
uint8_t UART6_rxBuffer[12] = {0};

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
		/* Enable DMA2 clock for data reception */
		//__HAL_RCC_DMA2_CLK_ENABLE();

    /* Configure peripheral GPIO for USART6 */
    /* PC6 ------> USART6_TX */
    /* PC7 ------> USART6_RX */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
		
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		/* Configure DMA stream */
    /* USART6_RX Init */
		/*
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_usart6_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart6_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart6_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart, hdmarx, hdma_usart6_rx);
		*/
	  /* Configure NVIC for DMA transfer complete interrupt (USART6_RX) */
		/*
		HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
		*/
		
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
		/* Disable USART6 DMA Stream */
    //HAL_DMA_DeInit(huart->hdmarx);
		/* Disable the NVIC for DMA */
		HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
  }
}

/**
  * @brief  This function handles DMA RX interrupt request.    
  */
/*
void USART6_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart6.hdmarx);
}
*/

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
		HAL_UART_Transmit(&huart6, UART6_rxBuffer, 1, 0xFF);
		
		if(HAL_UART_Receive_IT(&huart6, UART6_rxBuffer, 1) != HAL_OK)
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
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
	
	if(HAL_UART_Receive_IT(&huart6, UART6_rxBuffer, 1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Bluetooth_Send(uint8_t* data)
{
	HAL_UART_Transmit(&huart6, data, 12, 0xFF);
}