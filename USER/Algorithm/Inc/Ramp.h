/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ramp.c
  * @brief          : ramp functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RAMP_H
#define RAMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information  for the first order lowpass filter.
 */
typedef struct
{
    bool init;             /*!< init flag */
    float *filter_buff;    /*!< pointer to the floating-point array of filter buff */
    uint16_t length;       /*!< the length of filter buff */
    float input;           /*!< input value */
    float sum;             /*!< sum value */
    float output;          /*!< output value */
}MovingAverage_Info_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Calculate the floating-point ramp filter.
  */
extern float f_Ramp_Calc(float input,float target,float ramp);
/**
  * @brief Initializes the moving average filter according to the specified parameters in the
  *         MovingAverage_Info_TypeDef.
  */
extern void MovingAverage_Init(MovingAverage_Info_TypeDef *MA,uint16_t length);
/**
  * @brief update the floating-point moving average filter.
  */
extern float MovingAverage_Update(MovingAverage_Info_TypeDef *MA,float input);

#endif //RAMP_H

