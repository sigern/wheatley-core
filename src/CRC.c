#include "../include/CRC.h"

CRC_HandleTypeDef hcrc;

extern void Error_Handler(void);

void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance == CRC)
  {
    /* Peripheral clock enable */
    __HAL_RCC_CRC_CLK_ENABLE();
  }
}

void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance == CRC)
  {
    /* Peripheral clock disable */
    __HAL_RCC_CRC_CLK_DISABLE();
  }
}

uint32_t CRC_Calculate(uint32_t* buffer, uint32_t length)
{
	return HAL_CRC_Calculate(&hcrc, buffer, length);
}

void CRC_Init(void)
{
	hcrc.Instance = CRC;
	//hcrc.Instance->CR = CRC_CR_RESET | 0x00000080;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
    Error_Handler();
	}
}