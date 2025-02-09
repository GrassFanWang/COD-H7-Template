/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.h
  * @brief          : The header file of bsp_can.c 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to extern the functions and structure
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"

/**
 * @brief The structure that contains the Information of FDCAN Transmit.
 */
typedef struct{
		FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t				Data[8];
}FDCAN_TxFrame_TypeDef;

/**
 * @brief The structure that contains the Information of FDCAN Receive.
 */
typedef struct {
		FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			Data[8];
}FDCAN_RxFrame_TypeDef;

/* Externs ------------------------------------------------------------------*/
extern  FDCAN_TxFrame_TypeDef   FDCAN1_TxFrame;
extern  FDCAN_TxFrame_TypeDef   FDCAN2_TxFrame;
extern  FDCAN_TxFrame_TypeDef   FDCAN3_TxFrame;
extern void  USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame);
extern void BSP_FDCAN_Init(void);

	   
#endif