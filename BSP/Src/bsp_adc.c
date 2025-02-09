/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_adc.c
  * @brief          : bsp adc functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the adc
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_adc.h"
#include "adc.h"

/**
 * @brief ADC sampling voltage array
 */
__attribute__((section (".AXI_SRAM"))) uint16_t ADC_Voltage_Val[2];
 
/**
  * @brief  Configures the ADC. 
  * @param  None
  * @retval None
  */
void BSP_ADC_Init(void){

 	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Voltage_Val,2);

}
//------------------------------------------------------------------------------

/**
  * @brief  USER get current voltage. 
  * @param  None
  * @retval Voltage
  */
float USER_ADC_Voltage_Update(void){

  float Voltage;
	Voltage = (ADC_Voltage_Val[0]*3.3f/65535)*11.0f;
	return Voltage;
   
}   
//------------------------------------------------------------------------------













