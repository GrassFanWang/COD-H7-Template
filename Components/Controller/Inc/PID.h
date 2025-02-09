/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pid.c
  * @brief          : pid functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "lpf.h"
/* Exported defines -----------------------------------------------------------*/
/**
 * @brief macro definition of the VAL_LIMIT that restricts the value of the specified variable.
 * @param x: the specified variable
 * @param min: the minimum value of the specified variable
 * @param max: the maximum value of the specified variable
 * @retval none
 */
#define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)

/**
 * @brief macro definition of the number of pid parameters
*/
#ifndef PID_PARAMETER_NUM 
#define PID_PARAMETER_NUM 7							
#endif

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef enum that contains the Error status for the pid controller.
 */
typedef enum
{
    PID_ERROR_NONE = 0x00U,        /*!< No error */
    PID_FAILED_INIT = 0x01U,        /*!< Initialization failed */
		PID_CALC_NANINF = 0x02U,      /*!< Not-a-number (NaN) or infinity is generated */
    PID_Status_NUM,
}PID_Status_e;

/**
 * @brief typedef enum that contains the type for the pid controller.
 */
typedef enum
{
		PID_Type_None = 0x00U,         /*!< No Type */
		PID_POSITION = 0x01U,          /*!< position pid */
		PID_VELOCITY = 0x02U,          /*!< velocity pid */
    PID_TYPE_NUM,
}PID_Type_e;

/**
 * @brief typedef structure that contains the information for the pid Error handler.
 */
typedef struct
{
    uint16_t ErrorCount;    /*!< Error status judgment count */
    PID_Status_e Status;    /*!< Error Status */
}PID_ErrorHandler_Typedef;

/**
 * @brief typedef structure that contains the parameters for the pid controller.
 */
typedef struct
{
    float KP;             //比例系数
    float KI;             //积分系数
    float KD;             //微分系数
    float Alpha;           //微分一阶滤波器系数
		float Deadband;       //死区 当误差绝对值小于死区 PID停止计算。
    float LimitIntegral;  //积分限幅
    float LimitOutput;    //总输出限幅
}PID_Parameter_Typedef;

/**
 * @brief typedef structure that contains the information for the pid controller.
 */
typedef struct _PID_TypeDef
{
		PID_Type_e Type;    //PID类型 位置式or增量式 通常使用位置式
	
		float Target;       //目标值
		float Measure;      //实际值
	
    float Err[3];       //误差 目标值-期望值 = 误差 当前以及过去两次的误差 
		float Integral;     //误差积分值 误差累加
    float Pout;         // KP * 误差值 比例输出
    float Iout;         // KI * 误差积分 积分输出
    float Dout;         // KD * 误差微分（差分）微分输出
    float Output;       //总输出 Pout + Iout + Dout = Output
	
	  LowPassFilter1p_Info_TypeDef Dout_LPF; //微分输出的一阶滤波器
	
	
		PID_Parameter_Typedef Param;            //PID参数结构体
    PID_ErrorHandler_Typedef ERRORHandler;  //PID错误处理结构体

    /**
     * @brief 初始化PID参数的函数指针，将PID参数装载至PID参数结构体中。
     * @param PID: 指向_pid_TypeDef结构的指针，包含PID控制器的信息。
     * @param Param: 指向PID参数的浮点型指针，包含PID参数。
     * @retval PID错误状态 返回PID是否初始化成功。
     */
    PID_Status_e (*PID_Param_Init)(struct _PID_TypeDef *PID,float *Param);

    /**
     * @brief 清除pid计算函数的简短指针。
     * @param PID:指向_pid_TypeDef结构的指针，包含PID控制器的信息。
     * @retval 无.
     */
		void (*PID_Calc_Clear)(struct _PID_TypeDef *PID);
				
}PID_Info_TypeDef;


/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Initializes the PID Controller.
 */
extern void PID_Init(PID_Info_TypeDef *Pid,PID_Type_e type,float para[PID_PARAMETER_NUM]);
/**
  * @brief  Caculate the PID Controller
  */
extern float PID_Calculate(PID_Info_TypeDef *PID, float Target,float Measure);

#endif //CONTROLLER_PID_H


