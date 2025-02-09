/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_adc.h
  * @brief          : The header file of bsp_adc.h
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to config the clock source of the advanced TIM
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_ADC_H
#define BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* Externs ---------------------------------------------*/
void BSP_ADC_Init(void);
float USER_ADC_Voltage_Update(void);
#endif //BSP_PWM_H