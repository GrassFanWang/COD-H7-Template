/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : crc.h
  * @brief          : crc check 
  * @author         : GrassFan Wang
  * @date           : 2023/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef CRC_H
#define CRC_H

#include "stdint.h"
#include "stdbool.h"


extern uint8_t Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);

extern bool Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

extern void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

extern uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

extern bool Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

extern void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
#endif