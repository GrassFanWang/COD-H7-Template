/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the fdcan filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "bsp_can.h"
#include "Motor.h"
#include "Remote_Control.h"

/**
 * @brief The structure that contains the Information of FDCAN1 and FDCAN2 Receive.
 */
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;

/**
 * @brief The structure that contains the Information of FDCAN1 Transmit(CLASSIC_CAN).
 */
FDCAN_TxFrame_TypeDef FDCAN1_TxFrame = {
	.hcan = &hfdcan1,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

/**
 * @brief The structure that contains the Information of FDCAN2 Transmit(FDCAN).
 */
FDCAN_TxFrame_TypeDef FDCAN2_TxFrame = {
  .hcan = &hfdcan2,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_ON,
  .Header.FDFormat =  FDCAN_FD_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

/**
 * @brief The structure that contains the Information of FDCAN3 Transmit(CLASSIC_CAN).
 */
FDCAN_TxFrame_TypeDef FDCAN3_TxFrame = {
  .hcan = &hfdcan3,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
	.Header.MessageMarker = 0,
};

/**
  * @brief  Configures the FDCAN Filter. 
            FDCAN1:CLASSIC_CAN  FDCAN2:FDCAN  FDCAN3:CLASSIC_CAN
  * @param  None
  * @retval None
  */
void BSP_FDCAN_Init(void){

  FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID; // 过滤ID类型选择 标准ID
  FDCAN1_FilterConfig.FilterIndex = 0;           //当前FDCAN过滤器编号，可以设置多个过滤器过滤不同的ID 依次类推0、1、2....
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK; //过滤器Mask模式 关乎到下面ID1、ID2的配置
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//选择哪个FIFO区接收，根据CubeMX的配置来，FIFO1就改成FDCAN_FILTER_TO_RXFIFO1
  FDCAN1_FilterConfig.FilterID1 = 0x00000000; // 这个都行，只要ID2配置0x00000000就不会过滤调任何ID
  FDCAN1_FilterConfig.FilterID2 = 0x00000000; //理由如上
  
  HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig); //将上述配置到CAN1
		
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); //开启CAN1的全局过滤，就是开启过滤器
 
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//打开FIFO0区的新数据接收中断
  
  HAL_FDCAN_Start(&hfdcan1);//使能CAN1
 	
	
	FDCAN_FilterTypeDef FDCAN2_FilterConfig;
	
	FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN2_FilterConfig.FilterIndex = 0;
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  FDCAN2_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN2_FilterConfig.FilterID2 = 0x00000000; 
  
	HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2);//开启FDCAN的发送延迟补偿
 
  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2,14,14);//设置补偿时间 参数2和参数3都为TimeSeg1的值

  HAL_FDCAN_Start(&hfdcan2);
	
	
	FDCAN_FilterTypeDef FDCAN3_FilterConfig;
	
	FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN3_FilterConfig.FilterIndex = 0;
  FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN3_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN3_FilterConfig.FilterID2 = 0x00000000; 
  
	HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_FDCAN_Start(&hfdcan3);
}

/**
  * @brief  Function to transmit the FDCAN message.
  * @param  *FDCAN_TxFrame :the structure that contains the Information of FDCAN
  * @retval None
  */
void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame){

    HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame->hcan,&FDCAN_TxFrame->Header,FDCAN_TxFrame->Data);
 
}

/**
  * @brief  Function to converting the FDCAN1 received message to Fifo0.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN1_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8])
{
   
	DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[0]);
  DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[1]);
	DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[2]);
	DJI_Motor_Info_Update(Identifier,Data,&Chassis_Motor[3]);

}

/**
  * @brief  Function to converting the FDCAN3 received message to Fifo0.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN3_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8])
{



}


/**
  * @brief  Function to converting the FDCAN2 received message to Fifo1.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN2_RxFifo1RxHandler(uint32_t *Identifier,uint8_t Data[8])
{
	
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[0]);
  DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[1]);
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[2]);
	DM_Motor_Info_Update(Identifier,Data,&DM_8009_Motor[3]);
	

}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO0Frame.Header, FDCAN_RxFIFO0Frame.Data);
	
  if(hfdcan == &hfdcan1){	
	
   FDCAN1_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,FDCAN_RxFIFO0Frame.Data);
	 
	}

  if(hfdcan == &hfdcan3){
	
	 FDCAN3_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,FDCAN_RxFIFO0Frame.Data);
	
	}
	
}
	
/**
  * @brief  Rx FIFO 1 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxFIFO1Frame.Header, FDCAN_RxFIFO1Frame.Data);
	
  FDCAN2_RxFifo1RxHandler(&FDCAN_RxFIFO1Frame.Header.Identifier,FDCAN_RxFIFO1Frame.Data);
	 
}