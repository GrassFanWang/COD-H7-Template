/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_spi.h
  * @brief          : The header file of bsp_spi.h 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to extern the functions and structure
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_SPI_H
#define BSP_SPI_H


/* Include------------------------------------------------------------------*/
#include "stm32h7xx.h"

/* Externs ------------------------------------------------------------------*/
extern uint8_t BMI088_Read_Write_Byte(uint8_t Tx_Data);

#endif