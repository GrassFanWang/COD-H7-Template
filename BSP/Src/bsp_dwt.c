/**
  ******************************************************************************
  * @file           : bsp_dwt.c
  * @brief          : bsp DWT functions 
  * @author         : GrassFan Wang
  * @date           : 2025/02/17
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Include ---------------------------------------------------------*/
#include "bsp_dwt.h"
#include "stm32h723xx.h"

DWT_Time_Typedef SysTime;
uint64_t CYCCNT64;
/* Private variables ---------------------------------------------------------*/
static uint32_t CPU_FREQ_Hz;
static uint32_t CPU_FREQ_Hz_ms;
static uint32_t CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;

void BSP_DWT_Init(uint32_t CPU_Freq_mHz){

     /* Initializes the DWT peripheral */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* clear the DWT CYCCNT register */
    DWT->CYCCNT = (uint32_t)0u;

    /* Enable Cortex-M DWT CYCCNT register */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* update the clock frequence */
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;

}




