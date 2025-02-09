/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.h
  * @brief          : The header file of Motor.h 
  * @author         : GrassFan Wang
  * @date           : 2025/01/2
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H


/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32h723xx.h"
#include "bsp_can.h"

/**
 * @brief typedef enum that contains the type of DJI Motor Device.
 */
typedef enum{
    DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
    DJI_MOTOR_TYPE_NUM,
}DJI_Motor_Type_e;

/**
 * @brief  typedef enum that control mode the type of DM_Motor.
 */
typedef enum
{
  MIT,
	POSITION_VELOCITY,
	VELOCITY,
}DM_Motor_Control_Mode_Type_e;

/**
 * @brief  typedef enum that CMD of DM_Motor .
 */
typedef enum{
  Motor_Enable,
  Motor_Disable,
  Motor_Save_Zero_Position,
  DM_Motor_CMD_Type_Num,
}DM_Motor_CMD_Type_e;

/**
 * @brief typedef structure that contains the information for the motor FDCAN transmit and recieved .
 */
typedef struct
{
  uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier */
  uint32_t RxIdentifier;   /*!< Specifies FDCAN recieved identifier */

}Motor_CANFrameInfo_typedef;

/**
 * @brief typedef structure that contains the data for the Motor Device.
 */
typedef struct 
{
  bool Initlized;   /*!< init flag */
  int16_t  Current;   /*!< Motor electric current */
  int16_t  Velocity;    /*!< Motor rotate velocity (RPM)*/
  int16_t  Encoder;   /*!< Motor encoder angle */
  int16_t  Last_Encoder;   /*!< previous Motor encoder angle */
  float    Angle;   /*!< Motor angle in degree */
  uint8_t  Temperature;   /*!< Motor Temperature */
	
}DJI_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the param range for the DM_Motor .
 */
typedef struct 
{
  float  P_MAX;
	float  V_MAX;
	float  T_MAX;
}DM_Motor_Param_Range_Typedef;

/**
 * @brief typedef structure that contains the data for the DJI Motor Device.
 */
typedef struct 
{
	
  bool Initlized;    /*!< init flag */
  uint8_t  State; 	 /*!< Motor Message */
  uint16_t  P_int;   /*!< Motor Positon  uint16 */
	uint16_t  V_int;   /*!< Motor Velocity uint16 */
	uint16_t  T_int;   /*!< Motor Torque   uint16 */
	float  Position;   /*!< Motor Positon  */
  float  Velocity;   /*!< Motor Velocity */
  float  Torque;     /*!< Motor Torque   */
  float  Temperature_MOS;   /*!< Motor Temperature_MOS   */
	float  Temperature_Rotor; /*!< Motor Temperature_Rotor */
  float  Angle;	
	
}DM_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the information for the Damiao Motor Device.
 */
typedef struct
{
	DJI_Motor_Type_e Type;   /*!< Type of Motor */
  Motor_CANFrameInfo_typedef FDCANFrame;    /*!< information for the CAN Transfer */
	DJI_Motor_Data_Typedef Data;   /*!< information for the Motor Device */
}DJI_Motor_Info_Typedef;

/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct
{
  
	DM_Motor_Control_Mode_Type_e	Control_Mode;
  Motor_CANFrameInfo_typedef FDCANFrame;   
	DM_Motor_Param_Range_Typedef Param_Range; 
	DM_Motor_Data_Typedef Data;   

}DM_Motor_Info_Typedef;

/**
 * @brief typedef structure that contains the control information for the DM Motor Device .
 */
typedef struct
{
  float Position;
	float Velocity;
	float KP;
	float KD;
	float Torque;
	float Angle;
}DM_Motor_Contorl_Info_Typedef;

/* Externs ------------------------------------------------------------------*/
extern DJI_Motor_Info_Typedef DJI_Yaw_Motor,Chassis_Motor[4];

extern DM_Motor_Info_Typedef DM_8009_Motor[4];

extern DM_Motor_Contorl_Info_Typedef DM_Motor_Contorl_Info[4];

extern void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_Typedef *DJI_Motor);

extern void DM_Motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,DM_Motor_Info_Typedef *DM_Motor);

extern void DM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,uint8_t CMD);

extern void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque);

#endif //DEVICE_MOTOR_H
