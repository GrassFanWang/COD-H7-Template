/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_pwm.c
  * @brief          : bsp pwm functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to config the clock source of the advanced TIM
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_pwm.h"
#include "tim.h"

/**
  * @brief  Starts the PWM signal generation.
  * @param  None
  * @retval None
  */
void BSP_PWM_Init(void)
{
	//Heat_Power_Tim Start
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}
//------------------------------------------------------------------------------

/**
  * @brief  Set the BMI088 Heat_Power TIM Capture Compare Register value.
  * @param  compare specifies the Capture Compare register new value.
  * @retval None
  */
void Heat_Power_Control(uint16_t Compare)
{
    USER_Tim_SetCompare(&htim3,TIM_CHANNEL_4,Compare);
}
//------------------------------------------------------------------------------

/**
  * @brief  Set the TIM Capture Compare Register value.
  * @param  htim TIM PWM handle
  * @param  Channel TIM Channels to be configured
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  compare specifies the Capture Compare register new value.
  * @retval None
  */
void USER_Tim_SetCompare(TIM_HandleTypeDef *htim,uint32_t Channel,uint16_t Compare)
{
  switch (Channel)
  {
    case TIM_CHANNEL_1:
      htim->Instance->CCR1 = Compare;
    break;

    case TIM_CHANNEL_2:
      htim->Instance->CCR2 = Compare;
    break;

    case TIM_CHANNEL_3:
      htim->Instance->CCR3 = Compare;
    break;

    case TIM_CHANNEL_4:
      htim->Instance->CCR4 = Compare;
    break;

    default:break;
  }
}