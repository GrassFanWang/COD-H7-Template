/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_gpio.h
  * @brief          : The header file of bsp_can.h 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Externs ------------------------------------------------------------------*/
extern void BSP_GPIO_Init(void);

extern void BMI088_ACCEL_NS_L(void);

extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);

extern void BMI088_GYRO_NS_H(void);

#endif 