#include "Quaternion.h"
#include "Kalman_Filter.h"
#include "arm_math.h"


static float Fast_InverseSqrt(float x);

static void QuaternionEKF_A_Update(KalmanFilter_Info_TypeDef *KF);

static void QuaternionEKF_H_Update(KalmanFilter_Info_TypeDef *KF);

static bool QuaternionEKF_ChiSqrtTest(KalmanFilter_Info_TypeDef *KF);

static void QuaternionEKF_xhat_Update(KalmanFilter_Info_TypeDef *KF);

Quaternion_Info_Typedef Quaternion_Info;

void QuaternionEKF_Init(Quaternion_Info_Typedef *Quat,float Process_Noise1,float Process_Noise2,float Measure_Noise,float *QuaternionEKF_A_Data,float *QuaternionEKF_P_Data)
{

  Quat->Q1 = Process_Noise1;
  Quat->Q2 = Process_Noise2;
  Quat->R  = Measure_Noise;
                     
  Quat->QuaternionEKF_A_Data = QuaternionEKF_A_Data;
  Quat->QuaternionEKF_P_Data = QuaternionEKF_P_Data;

  Kalman_Filter_Init(&Quat->QuaternionEKF,6,0,3);
	
	Quat->QuaternionEKF.ChiSquareTest.TestFlag = false;
  Quat->QuaternionEKF.ChiSquareTest.Result = false;
  Quat->QuaternionEKF.ChiSquareTest.ChiSquareTestThresholds = 1e-8f;
  Quat->QuaternionEKF.ChiSquareTest.ChiSquareCnt = 0;
	
	Quat->QuaternionEKF.Data.Xhat[0] = 1.f;
  Quat->QuaternionEKF.Data.Xhat[1] = 0.f;
  Quat->QuaternionEKF.Data.Xhat[2] = 0.f;
  Quat->QuaternionEKF.Data.Xhat[3] = 0.f;
  
  Quat->QuaternionEKF.User_Function1 = QuaternionEKF_A_Update;
  Quat->QuaternionEKF.User_Function2 = QuaternionEKF_H_Update;
  Quat->QuaternionEKF.User_Function3 = QuaternionEKF_xhat_Update;

  Quat->QuaternionEKF.SkipStep3 = true;
  Quat->QuaternionEKF.SkipStep4 = true;

  memcpy(Quat->QuaternionEKF.Data.A,Quat->QuaternionEKF_A_Data,Quat->QuaternionEKF.sizeof_float * Quat->QuaternionEKF.Xhat_Size * Quat->QuaternionEKF.Xhat_Size);
  memcpy(Quat->QuaternionEKF.Data.P,Quat->QuaternionEKF_P_Data,Quat->QuaternionEKF.sizeof_float * Quat->QuaternionEKF.Xhat_Size * Quat->QuaternionEKF.Xhat_Size);

}

void QuaternionEKF_Update(Quaternion_Info_Typedef *Quat,float Gyro[3],float Accel[3],float dt)
{
  /* update cycle */
  Quat->QuaternionEKF.dt = dt;
  
	/* Apply accelerometer feedback to gyroscope */
	Quat->Gyro[0] = Gyro[0] - Quat->Deviate[0];
  Quat->Gyro[1] = Gyro[1] - Quat->Deviate[1];
  Quat->Gyro[2] = Gyro[2] - Quat->Deviate[2];
	
	Quat->GyroInvNorm = Fast_InverseSqrt(Quat->Gyro[0]*Quat->Gyro[0] + Quat->Gyro[1]*Quat->Gyro[1] + Quat->Gyro[2]*Quat->Gyro[2]);
	
  /* Convert gyroscope to radians per second scaled by 0.5 */	
	Quat->Half_Gyro_dt[0] = 0.5f * Quat->Gyro[0] * Quat->QuaternionEKF.dt;
	Quat->Half_Gyro_dt[1] = 0.5f * Quat->Gyro[1] * Quat->QuaternionEKF.dt;
  Quat->Half_Gyro_dt[2] = 0.5f * Quat->Gyro[2] * Quat->QuaternionEKF.dt;

 /**
   * @brief A = \frac{\partial f}{\partial x}
   *        (1,        -halfgxdt,  -halfgydt,  -halfgzdt,)   0.5f*q1*dt,  0.5f*q2*dt
   *        (halfgxdt,  1,          halfgzdt,  -halfgydt,)  -0.5f*q0*dt,  0.5f*q3*dt
   *        (halfgydt, -halfgzdt,   1,          halfgxdt,)  -0.5f*q3*dt, -0.5f*q0*dt
   *        (halfgzdt,  halfgydt,  -halfgxdt,   1,       )   0.5f*q2*dt, -0.5f*q1*dt
   *        (0,         0,          0,          0,         1,            0 )
   *        (0,         0,          0,          0,         0,            1 )
 */

  memcpy(Quat->QuaternionEKF.Data.A,Quat->QuaternionEKF_A_Data,Quat->QuaternionEKF.sizeof_float * Quat->QuaternionEKF.Xhat_Size * Quat->QuaternionEKF.Xhat_Size);

  Quat->QuaternionEKF.Data.A[1]  =  -Quat->Half_Gyro_dt[0];
	Quat->QuaternionEKF.Data.A[2]  =  -Quat->Half_Gyro_dt[1];
	Quat->QuaternionEKF.Data.A[3]  =  -Quat->Half_Gyro_dt[2];
	
	Quat->QuaternionEKF.Data.A[6]  =   Quat->Half_Gyro_dt[0];
	Quat->QuaternionEKF.Data.A[8]  =   Quat->Half_Gyro_dt[2];
	Quat->QuaternionEKF.Data.A[9]  =  -Quat->Half_Gyro_dt[1];

  Quat->QuaternionEKF.Data.A[12] =   Quat->Half_Gyro_dt[1];
	Quat->QuaternionEKF.Data.A[13] =  -Quat->Half_Gyro_dt[2];
	Quat->QuaternionEKF.Data.A[15] =   Quat->Half_Gyro_dt[0]; 

  Quat->QuaternionEKF.Data.A[18] =   Quat->Half_Gyro_dt[2];
	Quat->QuaternionEKF.Data.A[19] =   Quat->Half_Gyro_dt[1];
	Quat->QuaternionEKF.Data.A[20] =  -Quat->Half_Gyro_dt[0];

  memcpy(Quat->Accel,Accel,sizeof(Quat->Accel));

 /* Calculate direction of gravity indicated by measurement */
  Quat->AccelInvNorm = Fast_InverseSqrt(Quat->Accel[0] * Quat->Accel[0] + Quat->Accel[1] * Quat->Accel[1] + Quat->Accel[2] * Quat->Accel[2]);

  Quat->QuaternionEKF.MeasuredVector[0] =  Quat->Accel[0] * Quat->AccelInvNorm;
  Quat->QuaternionEKF.MeasuredVector[1] =  Quat->Accel[1] * Quat->AccelInvNorm;
  Quat->QuaternionEKF.MeasuredVector[2] =  Quat->Accel[2] * Quat->AccelInvNorm;
 
  /* chi square test */
  if(1.f/Quat->GyroInvNorm < 0.3f && 1.f/Quat->AccelInvNorm  > (GravityAccel-0.5f) && 1.f/Quat->AccelInvNorm < (GravityAccel+0.5f))
	{
		Quat->QuaternionEKF.ChiSquareTest.TestFlag = true;
  }
  else
  {
    Quat->QuaternionEKF.ChiSquareTest.TestFlag = false;
  }
  
  /* update the process/measurement noise covariance matrix */
  Quat->QuaternionEKF.Data.Q[0]  = Quat->Q1 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.Q[7]  = Quat->Q1 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.Q[14] = Quat->Q1 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.Q[21] = Quat->Q1 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.Q[28] = Quat->Q2 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.Q[35] = Quat->Q2 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.R[0]  = Quat->R;
  Quat->QuaternionEKF.Data.R[4]  = Quat->R;
  Quat->QuaternionEKF.Data.R[8]  = Quat->R;

  /* update the kalman filter */
  Kalman_Filter_Update(&Quat->QuaternionEKF);

  /* Update the Euler angle in radians */
  Quat->quat[0]    = Quat->QuaternionEKF.Output[0];
  Quat->quat[1]    = Quat->QuaternionEKF.Output[1];
  Quat->quat[2]    = Quat->QuaternionEKF.Output[2];
  Quat->quat[3]    = Quat->QuaternionEKF.Output[3];
  Quat->Deviate[0] = Quat->QuaternionEKF.Output[4];
  Quat->Deviate[1] = Quat->QuaternionEKF.Output[5];
  Quat->Deviate[2] = 0.f;
	
	
	/* Update the Euler angle in radians */
	Quat->EulerAngle[0] = atan2f(2.f*(Quat->quat[0]*Quat->quat[3] + Quat->quat[1]*Quat->quat[2]), 1.f - 2.f*(Quat->quat[2]*Quat->quat[2] + Quat->quat[3]*Quat->quat[3]));
  Quat->EulerAngle[1] = asinf( 2.f*(Quat->quat[0]*Quat->quat[2]) - 2.f*(Quat->quat[1]*Quat->quat[3] ));
	Quat->EulerAngle[2] = atan2f(2.f*(Quat->quat[0]*Quat->quat[1] + Quat->quat[2]*Quat->quat[3]), 1.f - 2.f*(Quat->quat[1]*Quat->quat[1] + Quat->quat[2]*Quat->quat[2]));
	
}

static void QuaternionEKF_A_Update(KalmanFilter_Info_TypeDef *KF){
   /* Normalise quaternion */
	memset(KF->Data.Cache_Vector[0],0,KF->sizeof_float * KF->Xhat_Size);
	KF->Data.Cache_Vector[0][0] = Fast_InverseSqrt(KF->Data.Xhatminus[0]*KF->Data.Xhatminus[0] \
                                                +KF->Data.Xhatminus[1]*KF->Data.Xhatminus[1] \
                                                +KF->Data.Xhatminus[2]*KF->Data.Xhatminus[2] \
                                                +KF->Data.Xhatminus[3]*KF->Data.Xhatminus[3]);
	
	KF->Data.Xhatminus[0] *= KF->Data.Cache_Vector[0][0];
  KF->Data.Xhatminus[1] *= KF->Data.Cache_Vector[0][0];
  KF->Data.Xhatminus[2] *= KF->Data.Cache_Vector[0][0];
  KF->Data.Xhatminus[3] *= KF->Data.Cache_Vector[0][0];
	
	KF->Data.A[4]  =  0.5f*KF->Data.Xhatminus[1]*KF->dt;
  KF->Data.A[5]  =  0.5f*KF->Data.Xhatminus[2]*KF->dt;

  KF->Data.A[10] = -0.5f*KF->Data.Xhatminus[0]*KF->dt;
  KF->Data.A[11] =  0.5f*KF->Data.Xhatminus[3]*KF->dt;

  KF->Data.A[16] = -0.5f*KF->Data.Xhatminus[3]*KF->dt;
  KF->Data.A[17] = -0.5f*KF->Data.Xhatminus[0]*KF->dt;

  KF->Data.A[22] =  0.5f*KF->Data.Xhatminus[2]*KF->dt;
  KF->Data.A[23] = -0.5f*KF->Data.Xhatminus[1]*KF->dt;

  /* Limit the P data */
  VAL_LIMIT(KF->Data.P[28],-10000,10000);
  VAL_LIMIT(KF->Data.P[35],-10000,10000);
	
}

static void QuaternionEKF_H_Update(KalmanFilter_Info_TypeDef *KF){
  
	memset(KF->Data.H,0,KF->sizeof_float * KF->Z_Size * KF->Xhat_Size);

  KF->Data.H[0]   = -2.f*KF->Data.Xhatminus[2];
  KF->Data.H[1]   =  2.f*KF->Data.Xhatminus[3];
  KF->Data.H[2]   = -2.f*KF->Data.Xhatminus[0];
  KF->Data.H[3]   =  2.f*KF->Data.Xhatminus[1];

  
	KF->Data.H[6]   =  2.f*KF->Data.Xhatminus[1];
  KF->Data.H[7]   =  2.f*KF->Data.Xhatminus[0];
  KF->Data.H[8]   =  2.f*KF->Data.Xhatminus[3];
  KF->Data.H[9]   =  2.f*KF->Data.Xhatminus[2];

	
	KF->Data.H[12]  =  2.f*KF->Data.Xhatminus[0];
  KF->Data.H[13]  = -2.f*KF->Data.Xhatminus[1];
  KF->Data.H[14]  = -2.f*KF->Data.Xhatminus[2];
  KF->Data.H[15]  =  2.f*KF->Data.Xhatminus[3];

	
	
}

static bool QuaternionEKF_ChiSqrtTest(KalmanFilter_Info_TypeDef *KF){
 
  // Cache_Matrix[0] = inverse(H，Pminus，HT + R)，(z(k) - h(Xhatminus))
  KF->Mat.Cache_Matrix[0].numRows = KF->Mat.Cache_Matrix[1].numRows;
  KF->Mat.Cache_Matrix[0].numCols = 1;
  KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[1], &KF->Mat.Cache_Vector[1], &KF->Mat.Cache_Matrix[0]);

 // Cache_Vector[0] = (z(k) - h(Xhatminus)'
	KF->Mat.Cache_Vector[0].numRows = 1;
  KF->Mat.Cache_Vector[0].numCols = KF->Mat.Cache_Matrix[1].numRows;
  KF->MatStatus = Matrix_Transpose(&KF->Mat.Cache_Matrix[1], &KF->Mat.Cache_Vector[0]);
	
// ChiSquare_Matrix = (z(k) - h(Xhatminus)'，inverse(H，Pminus，HT + R)，(z(k) - h(Xhatminus)) */
  KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Vector[0], &KF->Mat.Cache_Matrix[0], &KF->ChiSquareTest.ChiSquare_Matrix);
	
	if (KF->ChiSquareTest.ChiSquare_Data[0] < 0.5f * KF->ChiSquareTest.ChiSquareTestThresholds)
  {
      KF->ChiSquareTest.Result = true;
  }
	
	  if (KF->ChiSquareTest.ChiSquare_Data[0] > KF->ChiSquareTest.ChiSquareTestThresholds && KF->ChiSquareTest.Result)
  {
      if (KF->ChiSquareTest.TestFlag)
      {
          KF->ChiSquareTest.ChiSquareCnt++;
      }
      else
      {
          KF->ChiSquareTest.ChiSquareCnt = 0;
      }

      if (KF->ChiSquareTest.ChiSquareCnt > 50)
      {
          KF->ChiSquareTest.Result = 0;
          KF->SkipStep5 = false; // step-5 is cov mat P updating
      }
      else
      {
          /* xhat(k) = xhat'(k) */
          /* P(k) = P'(k) */
          memcpy(KF->Data.Xhat, KF->Data.Xhatminus, KF->sizeof_float * KF->Xhat_Size);
          memcpy(KF->Data.P, KF->Data.Pminus, KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);

          /* skip the P update */
          KF->SkipStep5 = true;
          return true;
      }
  }else{
			
		 /* The smaller the rk , the greater the gain */
			if(KF->ChiSquareTest.ChiSquare_Data[0] > 0.1f * KF->ChiSquareTest.ChiSquareTestThresholds && KF->ChiSquareTest.Result)
			{
				KF->Data.Cache_Vector[0][0] = (KF->ChiSquareTest.ChiSquareTestThresholds - KF->ChiSquareTest.ChiSquare_Data[0]) / (0.9f * KF->ChiSquareTest.ChiSquareTestThresholds);
			}
			else
			{
				KF->Data.Cache_Vector[0][0] = 1.f;
			}
			
      KF->ChiSquareTest.ChiSquareCnt = 0;
      KF->SkipStep5 = false;
  }
  return false;
}


static void QuaternionEKF_xhat_Update(KalmanFilter_Info_TypeDef *KF){

	  /* HT */
  KF->MatStatus = Matrix_Transpose(&KF->Mat.H,&KF->Mat.HT);

  /* Cache_Matrix[0] = H，Pminus */
  KF->Mat.Cache_Matrix[0].numRows = KF->Mat.H.numRows;
  KF->Mat.Cache_Matrix[0].numCols = KF->Mat.Pminus.numCols;
  KF->MatStatus = Matrix_Multiply(&KF->Mat.H, &KF->Mat.Pminus, &KF->Mat.Cache_Matrix[0]);

  /* Cache_Matrix[1] = H，Pminus，HT */
  KF->Mat.Cache_Matrix[1].numRows = KF->Mat.Cache_Matrix[0].numRows;
  KF->Mat.Cache_Matrix[1].numCols = KF->Mat.HT.numCols;
  KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.HT, &KF->Mat.Cache_Matrix[1]); 
  
  /* K_denominator = H Pminus HT + R */
  KF->Mat.K_denominator.numRows = KF->Mat.R.numRows;
  KF->Mat.K_denominator.numCols = KF->Mat.R.numCols;
  KF->MatStatus = Matrix_Add(&KF->Mat.Cache_Matrix[1], &KF->Mat.R, &KF->Mat.K_denominator);

  /* Cache_Matrix[1] = inverse(H，Pminus，HT + R) */
  KF->MatStatus = Matrix_Inverse(&KF->Mat.K_denominator, &KF->Mat.Cache_Matrix[1]);
	
  KF->Mat.Cache_Vector[0].numRows = KF->Mat.H.numRows;
  KF->Mat.Cache_Vector[0].numCols = 1;
  KF->Data.Cache_Vector[0][0] = 2.f * (KF->Data.Xhatminus[1] * KF->Data.Xhatminus[3] - KF->Data.Xhatminus[0] * KF->Data.Xhatminus[2]);
  KF->Data.Cache_Vector[0][1] = 2.f * (KF->Data.Xhatminus[0] * KF->Data.Xhatminus[1] + KF->Data.Xhatminus[2] * KF->Data.Xhatminus[3]);
  KF->Data.Cache_Vector[0][2] =   KF->Data.Xhatminus[0] * KF->Data.Xhatminus[0] \
                                - KF->Data.Xhatminus[1] * KF->Data.Xhatminus[1] \
                                - KF->Data.Xhatminus[2] * KF->Data.Xhatminus[2] \
                                + KF->Data.Xhatminus[3] * KF->Data.Xhatminus[3];
																
	/* the cosine of three axis orientation */
	float OrientationCosine[3];
	
  /* calculate the cosine of three axis orientation */
	for (uint8_t i = 0; i < 3; i++)
	{
		OrientationCosine[i] = acosf(fabsf(KF->Data.Cache_Vector[0][i]));
	}
																
	 /* Cache_Vector[1] = z(k) - h(xhat'(k)) */
  KF->Mat.Cache_Vector[1].numRows = KF->Mat.Z.numRows;
  KF->Mat.Cache_Vector[1].numCols = 1;
  KF->MatStatus = Matrix_Subtract(&KF->Mat.Z, &KF->Mat.Cache_Vector[0], &KF->Mat.Cache_Vector[1]);

  /* Chi Square root Test */
  if(QuaternionEKF_ChiSqrtTest(KF)==true)
  {
    return;
  }

  /* Cache_Matrix[0] = Pminus，HT */
  KF->Mat.Cache_Matrix[0].numRows = KF->Mat.Pminus.numRows;
  KF->Mat.Cache_Matrix[0].numCols = KF->Mat.HT.numCols;
  KF->MatStatus = Matrix_Multiply(&KF->Mat.Pminus, &KF->Mat.HT, &KF->Mat.Cache_Matrix[0]);

  /* k = Pminus，HT，inverse(H，Pminus，HT + R) */
  KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.Cache_Matrix[1], &KF->Mat.K);
	
	/* The smaller the rk , the greater the gain */
	for(uint8_t i = 0; i < KF->Mat.K.numCols*KF->Mat.K.numRows; i++)
	{
		KF->Data.K[i] *= KF->Data.Cache_Vector[0][0];
	}

  /**
   * @brief K = \frac {P，minus，HT}{H，Pminus，HT + V，R，VT}
   *          = [  0,  1,  2,
   *               3,  4,  5,
   *               6,  7,  8,
   *               9, 10, 11, 
   *             (12, 13, 14,)
   *             (15, 16, 17,)]
   * @note  K[12..17] *=  cos(axis)/(PI/2.f)
   */
  for (uint8_t i = 4; i < 6; i++)
  {
    for (uint8_t j = 0; j < 3; j++)
    {
        KF->Data.K[i * 3 + j] *= OrientationCosine[i - 4] / 1.5707963f; // 1 rad
    }
  }

  /* Cache_Vector[0] = K(k)，(z(k) - H，xhat'(k)) */
  KF->Mat.Cache_Vector[0].numRows = KF->Mat.K.numRows;
  KF->Mat.Cache_Vector[0].numCols = 1;
  KF->MatStatus = Matrix_Multiply(&KF->Mat.K, &KF->Mat.Cache_Vector[1], &KF->Mat.Cache_Vector[0]);

  if(KF->ChiSquareTest.Result)
  {
    VAL_LIMIT(KF->Data.Cache_Vector[0][4],-1e-2f*KF->dt,1e-2f*KF->dt);
    VAL_LIMIT(KF->Data.Cache_Vector[0][5],-1e-2f*KF->dt,1e-2f*KF->dt);
  }
  KF->Data.Cache_Vector[0][3] = 0;

  KF->MatStatus = Matrix_Add(&KF->Mat.Xhatminus, &KF->Mat.Cache_Vector[0], &KF->Mat.Xhat);															
																											
}

static float Fast_InverseSqrt(float x)
{

    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;

    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}