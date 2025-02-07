/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : GrassFam Wang
  * @date           : 2025/1/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "Control_Task.h"

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */

 void CAN_Task(void const * argument)
{

 
  TickType_t CAN_Task_SysTick = 0;
	DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[0],Motor_Enable);
  osDelay(30);
	DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[1],Motor_Enable);
  osDelay(30);
  DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[2],Motor_Enable);
  osDelay(30);
	DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[4],Motor_Enable);
  osDelay(30);
	for(;;)
  {
	
  CAN_Task_SysTick = osKernelSysTick();
		
	 // CAN-FD	
	 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,0,0,0);
	 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,0,0,0);
   DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,0,0,0);	
   DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,0,0,0);	
		
   FDCAN1_TxFrame.Data[0] = (uint8_t)(Control_Info.SendValue[0]>>8);	
	 FDCAN1_TxFrame.Data[1] = (uint8_t)(Control_Info.SendValue[0]);	
	 FDCAN1_TxFrame.Data[2] = (uint8_t)(Control_Info.SendValue[1]>>8);	
	 FDCAN1_TxFrame.Data[3] = (uint8_t)(Control_Info.SendValue[1]);	
   USER_FDCAN_AddMessageToTxFifoQ(&FDCAN1_TxFrame);
		
	 if(CAN_Task_SysTick % 2 == 0){
	 
	 //500Hz发送 请保证所有任务osDelay(1)
	 
	 }	
		osDelay(1);
  }
 
}


