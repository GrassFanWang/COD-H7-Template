/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Image_Transmission.h
  * @brief          : The header file of Image_Transmission.c
  * @author         : GrassFan Wang
  * @date           : 2025/04/20
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef IMAGE_TRANSMISSION_H
#define IMAGE_TRANSMISSION_H
  
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <string.h>

#define IMAGE_TRANS_RX_LENGTH  39  //frame_header 5bytes , cmd_id 2bytes , data_max 30bytes , crc16 2bytes = bytes

#define FrameHeader_Length    5U   /*!< the length of frame header */
#define CMDID_Length          2U   /*!< the length of CMD ID */
#define CRC16_Length          2U   /*!< the length of CRC ID */

#define CUSTOM_ROBOT_DATA_ID        0x0302U  
#define ROBOT_CUSTOM_DATA_ID        0x0309U  
#define REMOTE_CONTROL_ID           0x0304U  

/**
 * @brief The custom controller sends data to the corresponding robot through the image transmission, id: 0x0302U
 */
typedef struct{

  uint8_t data[30];

}custom_robot_data_t;

/**
 * @brief The robot sends data to the custom controller connected to the player end through the image transmission, id: 0x0309U
 */
typedef struct{

  uint8_t data[30];

}robot_custom_data_t;

/**
 * @brief The keyboard and mouse remote control data sent to the robot through the image transmission, id: 0x0304U
 */
typedef struct{

	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
 union
	 {
		uint16_t keyboard_value;
		struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		} Set;
	} Key;
	 
	uint16_t reserved;

}remote_control_t;

typedef struct{

	uint8_t Index;
  uint16_t DataLength;
	
	#ifdef CUSTOM_ROBOT_DATA_ID
	  custom_robot_data_t custom_robot_data;
	#endif
	
	#ifdef ROBOT_CUSTOM_DATA_ID
	  robot_custom_data_t robot_custom_data;
	#endif
	
	#ifdef REMOTE_CONTROL_ID
	  remote_control_t remote_control;
  #endif
}Image_Transmission_Info_TypeDef;


typedef struct{
     
   struct
	 {
		int16_t Channel[4];
	  uint8_t Switch;
	  uint8_t Stop;
	  uint8_t Left;
	  uint8_t Right;
	  int16_t Wheel;
	  uint8_t Trigger;
	 }RC;
	 
		struct
	 {
			int16_t X;
			int16_t Y;
			int16_t Z;
			uint8_t Press_L;
			uint8_t Press_R;
			uint8_t Press_M;
	 } Mouse;
	  
   union
	 {
		uint16_t V;
		struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		} Set;
	} Key;
	
}VT13_Info_TypeDef;

/* Exported variables ---------------------------------------------------------*/
extern uint8_t Image_Trans_MultiRx_Buff[2][39];
extern VT13_Info_TypeDef VT13_Info;
extern Image_Transmission_Info_TypeDef Image_Transmission_Info;
extern void Robot_Data_to_Custom_(uint8_t *Data);
extern void Image_Transmission_Info_Update(uint8_t *Buff);
extern void VT13_Info_Update(uint8_t *Data ,VT13_Info_TypeDef *VT13_Info);

#endif