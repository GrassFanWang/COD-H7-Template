#ifndef BSP_DWT_H
#define BSP_DWT_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the DWT timer.
 */
typedef struct
{
  uint32_t s;
  uint16_t ms;
  uint16_t us;
} DWT_Time_Typedef;


extern void BSP_DWT_Init(uint32_t CPU_Freq_mHz);

#endif