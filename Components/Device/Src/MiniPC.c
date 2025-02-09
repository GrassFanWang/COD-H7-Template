/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.c
  * @brief          : MiniPC interfaces functions 
  * @author         : GarssFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "MiniPC.h"
#include "usbd_cdc_if.h"


void MiniPC_Transmit_Info(uint8_t *Buff){

    CDC_Transmit_HS(Buff,sizeof(*Buff));

}

//usbd_cdc_if.c -> CDC_Receive_HS
void MiniPC_Recvive_Info(uint8_t* Buff, const uint32_t *Len){



}