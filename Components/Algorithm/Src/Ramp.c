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

/* Includes ------------------------------------------------------------------*/
#include "ramp.h"

/* Private define ------------------------------------------------------------*/
/**
  * @brief Calculate the floating-point ramp filter.
  * @param input: the filter input variables
  * @param target: the input variables target value
  * @param ramp: the ramp slope
  * @retval the filter output
  */
float f_Ramp_Calc(float input,float target,float ramp)
{
  float error = target - input;
  float output = input;

	if (error > 0){
        if (error > ramp){output += ramp;}   
        else{output += error;}
    }else{
        if (error < -ramp){output += -ramp;}
        else{output += error;}
    }

    return output;
}
//------------------------------------------------------------------------------

/**
  * @brief Initializes the moving average filter according to the specified parameters in the
  *         MovingAverage_Info_TypeDef.
  * @param MA: pointer to an MovingAverage_Info_TypeDef structure that
  *         contains the information  for the moving average filter.
  * @param length: the length of filter buff
  * @retval none
  */
void MovingAverage_Init(MovingAverage_Info_TypeDef *MA,uint16_t length)
{

  MA->length = length;

  MA->filter_buff = malloc(sizeof(float)*MA->length);
  memset(MA->filter_buff,0,sizeof(float)*MA->length);

  if(MA->filter_buff == NULL)
  {
    return ;
  }

  MA->input = 0;
  MA->output = 0;

  MA->init = true;
}
//------------------------------------------------------------------------------


/**
  * @brief Calculate the floating-point moving average filter.
  * @param MA: pointer to an MovingAverage_Info_TypeDef structure that
  *         contains the information  for the moving average filter.
  * @param input: the input variable
  * @retval the filter output
  */
float MovingAverage_Update(MovingAverage_Info_TypeDef *MA,float input)
{
  if(MA->init != true)
  {
    return 0;
  }

  /* moving the filter buff */
  for(uint16_t i = 0; i < MA->length-1; i++)
  {
      MA->filter_buff[i+1] = MA->filter_buff[i];
  }

  /* update the filter input */
  MA->filter_buff[0] = input;

  /* calculate the average */
  for(uint16_t i = 0; i < MA->length-1; i++)
  {
      MA->sum += MA->filter_buff[i];
  }

  MA->output = (float)(MA->sum / MA->length);
	
	MA->sum = 0;

  return MA->output;
}
//------------------------------------------------------------------------------


