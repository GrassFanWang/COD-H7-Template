/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lowpass_filter.c
  * @brief          : lowpass filter 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  一阶低通滤波器信息的结构体
 */
typedef struct
{
	bool Initialized;       //初始化标志 0:未初始化 1：初始化
    float Input;          //当前输入
    float Output;         //输出
    float Alpha;          //滤波系数
}LowPassFilter1p_Info_TypeDef;

/**
 * @brief 二阶低通滤波器信息的结构体.
 */

typedef struct 
{
    bool Initialized;  //初始化标志 0:未初始化 1：初始化
    float Input;       //当前输入
    float Output[3];   //输出 现在和过去两次的输出
    float Alpha[3];    //二阶滤波器系数

}LowPassFilter2p_Info_TypeDef;

/* Extern Functions Prototypes ---------------------------------------------*/

/**
  * @brief 根据函数中指定的参数初始化一阶低通滤波器.
  */
extern void LowPassFilter1p_Init(LowPassFilter1p_Info_TypeDef *LPF,float Alpha);
/**
  * @brief 根据函数中指定的参数更新（计算）一阶低通滤波器
  */
extern float LowPassFilter1p_Update(LowPassFilter1p_Info_TypeDef *lpf,float input);
/**
  * @brief 根据函数中指定的参数初始化二阶低通滤波器
  */
extern void  LowPassFilter2p_Init(LowPassFilter2p_Info_TypeDef *lpf,float alpha[3]);
/**
  * @brief 根据函数中指定的参数更新（计算）二阶低通滤波器
  */
extern float LowPassFilter2p_Update(LowPassFilter2p_Info_TypeDef *lpf,float input);

extern float sign(float input);
#endif //LOWPASS_FILTER_H
