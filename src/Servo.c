#include "../include/Servo.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

extern void Error_Handler(uint8_t val);

/**
* @brief TIM_PWM MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_pwm: TIM_PWM handle pointer
* @retval None
*/
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance == TIM2) {
    /* Peripheral TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  } else if(htim_pwm->Instance == TIM5) {
    /* Peripheral TIM5 clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance == TIM2) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**
		   TIM2 GPIO Configuration
       PA3 ------> TIM2_CH4
     */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  } else if(htim->Instance==TIM5) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**
		   TIM5 GPIO Configuration
       PA2 ------> TIM5_CH3
     */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance == TIM2) {
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  } else if(htim_base->Instance == TIM5) {
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();
  }

}

/* Init TIMER2 CHANNEL 4 (GPIO PA3) in PWM mode for Tilt Servo 

		Calculations for TIM4

		TIM_Period = Timer_default_frequency / (PWM_frequency * (prescaller_set + 1)) - 1
		
		For Tilt servo (digital servo):
			PWM_frequency = 350 Hz

		For TIM4 (APB1 bus):
			Timer_default_frequency = 84 000 000 Hz

		Prescaler set to:
			prescaller_set = 11
		
		TIM_Period = 84 000 000 /(350 * (11 + 1)) - 1 = 19 999
*/	
static void TIM2_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 11;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler(1);
  }
	
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler(1);
  }
  HAL_TIM_MspPostInit(&htim2);
}

/* Init TIMER5 CHANNEL 3 (GPIO PA2) in PWM mode for Roll Servo

		Calculations for TIM4

		TIM_Period = Timer_default_frequency / (PWM_frequency * (prescaller_set + 1)) - 1
		
		For Tilt servo (analog servo):
			PWM_frequency = 50 Hz

		For TIM4 (APB1 bus):
			Timer_default_frequency = 84 000 000 Hz

		Prescaler set to:
			prescaller_set = 41
		
		TIM_Period = 84 000 000 /(50 * (41 + 1)) - 1 = 39 999
*/	
static void TIM5_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 41;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 39999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler(1);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ROLL_ZERO;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler(1);
  }
	
  HAL_TIM_MspPostInit(&htim5);
}

void Servo_Init(void)
{
	/* Tilt servo init */
	TIM2_Init();
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK) {
		/* PWM Generation Error for TIM2_CH4 */
		Error_Handler(1);
	}
	ServoTilt_Set(TILT_ZERO);
	
	
	/* Roll servo init */
	TIM5_Init();
	if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) != HAL_OK) {
	  /* PWM Generation Error for TIM5_CH3 */
	  Error_Handler(1);
  }
	ServoRoll_Set(ROLL_ZERO);
}

void Servo_Enable(bool isEnable)
{
	if (isEnable) {
		__HAL_TIM_ENABLE(&htim2);
		__HAL_TIM_ENABLE(&htim5);
	} else {
		__HAL_TIM_DISABLE(&htim2);
		__HAL_TIM_DISABLE(&htim5);
	}
}

void ServoRoll_Set(uint16_t value)
{
	TIM5->CCR3 = value;
}

void ServoTilt_Set(uint16_t value)
{
	TIM2->CCR4 = value;
}
