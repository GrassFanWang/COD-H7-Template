/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Remote_Control.c
  * @brief          : remote_control interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
/* Exported defines -----------------------------------------------------------*/
/**
 * @brief Length of SBUS received data
 */
#define SBUS_RX_BUF_NUM		18u
/**
 * @brief offset of remote control channel data
 */
#define RC_CH_VALUE_OFFSET		1024U

/**
 * @brief judgement keyboard set short time
 */
#define KEY_SET_SHORT_TIME		50U
/**
 * @brief judgement keyboard set long time
 */
#define KEY_SET_LONG_TIME		1000U

/**
 * @brief status of keyboard up
 */
#define KEY_UP                    0x00U
/**
 * @brief status of keyboard down
 */
#define KEY_DOWN                  0x01U

/**
 * @brief MAX speed of mouse speed
 */
#define MOUSE_SPEED_MAX		300U

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef enum that contains the status of the keyboard.
 */
typedef enum
{
	UP,			/*!< up */
	SHORT_DOWN,	/*!< short time down */
	DOWN,		/*!< long time down */
	PRESS,		/*!< 0->1 */
	RELAX,		/*!< 1->0 */
	KeyBoard_Status_NUM,
}KeyBoard_Status_e;

typedef struct
{
	uint16_t Count;
	KeyBoard_Status_e Status;
	KeyBoard_Status_e last_Status;
	bool last_KEY_PRESS;
	bool KEY_PRESS;
}KeyBoard_Info_Typedef;

typedef struct
{
	KeyBoard_Info_Typedef press_l;
	KeyBoard_Info_Typedef press_r;
	KeyBoard_Info_Typedef W;
	KeyBoard_Info_Typedef S;
	KeyBoard_Info_Typedef A;
	KeyBoard_Info_Typedef D;
	KeyBoard_Info_Typedef SHIFT;
	KeyBoard_Info_Typedef CTRL;
	KeyBoard_Info_Typedef Q;
	KeyBoard_Info_Typedef E;
	KeyBoard_Info_Typedef R;
	KeyBoard_Info_Typedef F;
	KeyBoard_Info_Typedef G;
	KeyBoard_Info_Typedef Z;
	KeyBoard_Info_Typedef X;
	KeyBoard_Info_Typedef C;
	KeyBoard_Info_Typedef V;
	KeyBoard_Info_Typedef B;
}Remote_Pressed_Typedef;


/**
 * @brief typedef structure that contains the information for the remote control.
 */
typedef  struct
{
	/**
	 * @brief structure that contains the information for the lever/Switch.
	 */
	struct
	{
		int16_t ch[5];
		uint8_t s[2];
	} rc;
	
	/**
	 * @brief structure that contains the information for the mouse.
	 */
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	/**
	 * @brief structure that contains the information for the keyboard.
	 */
	union
	{
		uint16_t v;
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
		} set;
	} key;

	bool rc_lost;   /*!< lost flag */
	uint8_t online_cnt;   /*!< online count */
} Remote_Info_Typedef;

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
extern Remote_Info_Typedef remote_ctrl;
/**
 * @brief remote control usart RxDMA MultiBuffer
 */
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/* Mouse Exported defines -----------------------------------------------------*/
#define MOUSE_X_MOVE_SPEED    (remote_ctrl.mouse.x )
#define MOUSE_Y_MOVE_SPEED    (remote_ctrl.mouse.y )
#define MOUSE_Z_MOVE_SPEED    (remote_ctrl.mouse.z )
#define MOUSE_PRESSED_LEFT    (remote_ctrl.mouse.press_l)
#define MOUSE_PRESSED_RIGHT   (remote_ctrl.mouse.press_r)

/* KeyBoard Exported defines --------------------------------------------------*/
#define KeyBoard_W            (remote_ctrl.key.set.W)
#define KeyBoard_S            (remote_ctrl.key.set.S)
#define KeyBoard_A            (remote_ctrl.key.set.A)
#define KeyBoard_D            (remote_ctrl.key.set.D)
#define KeyBoard_SHIFT        (remote_ctrl.key.set.SHIFT)
#define KeyBoard_CTRL         (remote_ctrl.key.set.CTRL)
#define KeyBoard_Q            (remote_ctrl.key.set.Q)
#define KeyBoard_E            (remote_ctrl.key.set.E)
#define KeyBoard_R            (remote_ctrl.key.set.R)
#define KeyBoard_F            (remote_ctrl.key.set.F)
#define KeyBoard_G            (remote_ctrl.key.set.G)
#define KeyBoard_Z            (remote_ctrl.key.set.Z)
#define KeyBoard_X            (remote_ctrl.key.set.X)
#define KeyBoard_C            (remote_ctrl.key.set.C)
#define KeyBoard_V            (remote_ctrl.key.set.V)
#define KeyBoard_B            (remote_ctrl.key.set.B)

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  convert the remote control received message
  */
extern void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote_ctrl);
/**
  * @brief  clear the remote control data while the device offline
  */
extern void Remote_Message_Moniter(Remote_Info_Typedef *remote_ctrl);

/**
  * @brief  report the cover status that acrroding the key_R swicthing
  */
extern bool Key_R(void);
/**
  * @brief  switch the shooter mode that acrroding the key_B
  */
extern bool Key_B(void);

/**
  * @brief  report the auto aim status that acrroding the mouse right swicthing
  */
extern bool Mouse_Pressed_Right(void);
/**
  * @brief  report the fire status that acrroding the mouse left swicthing
  */
extern bool Mouse_Pressed_Left(void);

#endif //REMOTE_CONTROL_H


