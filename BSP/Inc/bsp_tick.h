/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_tick.h
  * @brief          : The header file of bsptick.h
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : use the TIM2 as the HAL TimeBase
  ******************************************************************************
  */
/* USER CODE END Header */


#ifndef BSP_TICK_H
#define BSP_TICK_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"



extern void Delay_us(uint32_t us);

extern void Delay_ms(uint32_t ms);
	
	
#endif