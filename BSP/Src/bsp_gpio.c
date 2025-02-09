/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_gpio.c
  * @brief          : bsp gpio functions 
  * @author         : GrassFan Wan
  * @date           : 2025/1/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_gpio.h"
#include "main.h"

/**
  * @brief  Configures the GPIO.
  * @param  None
  * @retval None
  */
void BSP_GPIO_Init(void){

//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
//  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);

}

/**
  * @brief RESET the BMI088_ACCEL_NS
  * @note GPIO_x: GPIOC
  * @note GPIO_PIN_x: GPIO_PIN_0
  */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port,ACCEL_CS_Pin,GPIO_PIN_RESET);
}

/**
  * @brief SET the BMI088_ACCEL_NS
  * @note GPIO_x: GPIOC
  * @note GPIO_PIN_x: GPIO_PIN_0
  */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACCEL_CS_GPIO_Port,ACCEL_CS_Pin,GPIO_PIN_SET);
}

/**
  * @brief RESET the BMI088_GYRO_NS
  * @note GPIO_x: GPIOC
  * @note GPIO_PIN_x: GPIO_PIN_3
  */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,GPIO_PIN_RESET);
}

/**
  * @brief RESET the BMI088_GYRO_NS
  * @note GPIO_x: GPIOC
  * @note GPIO_PIN_x: GPIO_PIN_3
  */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,GPIO_PIN_SET);
}