#ifndef QUATERNION_H
#define QUATERNION_H

#include "stdint.h"
#include "stdbool.h"
#include "Kalman_Filter.h"

typedef struct
{
  bool Init;
	
	float quat[4];
  float Deviate[3];
	
	float Q1,Q2,R;
	
  float *QuaternionEKF_A_Data;  /*!< pointer to the data of state transition matrix */
  float *QuaternionEKF_P_Data;  /*!< pointer to the data of posteriori covariance matrix */
  KalmanFilter_Info_TypeDef QuaternionEKF;  /*!< Extended Kalman Filter */

  float Accel[3];      /*!< the data of accel measure */
  float Gyro[3];       /*!< the data of gyro measure */
  float AccelInvNorm;       /*!< the inverse of accel norm */
  float GyroInvNorm;        /*!< the inverse of gyro norm */
  float Half_Gyro_dt[3];      /*!< half gyro dt */
  float EulerAngle[3];      /*!< Euler angles in radians: */

}Quaternion_Info_Typedef;


extern Quaternion_Info_Typedef Quaternion_Info;

extern void QuaternionEKF_Init(Quaternion_Info_Typedef *Quat,float Process_Noise1,float Process_Noise2,float Measure_Noise,float *QuaternionEKF_A_Data,float *QuaternionEKF_P_Data);

extern void QuaternionEKF_Update(Quaternion_Info_Typedef *Quat,float Gyro[3],float Accel[3],float dt);


#endif