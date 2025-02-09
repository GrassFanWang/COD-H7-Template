#include "Kalman_Filter.h"

void Kalman_Filter_Init(KalmanFilter_Info_TypeDef *KF,uint8_t Xhat_Size,uint8_t U_Size,uint8_t Z_Size)
{
    /* Update the size of float/float */
    KF->sizeof_float = sizeof(float);
    KF->sizeof_double = sizeof(double);

    /* judge the length of vector */
    if(Xhat_Size == 0 || Z_Size == 0)
    {
        KF->MatStatus = ARM_MATH_LENGTH_ERROR;  
    }
    
    /* Initializes the state vector dimension */
    KF->Xhat_Size = Xhat_Size;

     /* Initializes the control vector dimension */
    KF->U_Size = U_Size;

    /**< Initializes the measurement vector dimension */      
    KF->Z_Size = Z_Size;

    /* Initializes the ChiSquare matrix */
    memset(KF->ChiSquareTest.ChiSquare_Data,0,sizeof(KF->ChiSquareTest.ChiSquare_Data));
    Matrix_Init(&KF->ChiSquareTest.ChiSquare_Matrix, 1, 1, (float *)KF->ChiSquareTest.ChiSquare_Data);

    /* Initializes the external measure vector */
    KF->MeasuredVector = (float *)User_malloc(KF->sizeof_float * Z_Size);
    memset(KF->MeasuredVector, 0, KF->sizeof_float * Z_Size);

    /* Initializes the Xhat */
    KF->Data.Xhat = (float *)User_malloc(KF->sizeof_float * Xhat_Size);
    memset(KF->Data.Xhat, 0, KF->sizeof_float * Xhat_Size);
    Matrix_Init(&KF->Mat.Xhat, KF->Xhat_Size, 1, (float *)KF->Data.Xhat);

    /* Initializes the Xhatminus */
    KF->Data.Xhatminus = (float *)User_malloc(KF->sizeof_float * Xhat_Size);
    memset(KF->Data.Xhatminus, 0, KF->sizeof_float * Xhat_Size);
    Matrix_Init(&KF->Mat.Xhatminus, KF->Xhat_Size, 1, (float *)KF->Data.Xhatminus);

    /* Initializes the measurement vector */
    KF->Data.Z = (float *)User_malloc(KF->sizeof_float * Z_Size);
    memset(KF->Data.Z, 0, KF->sizeof_float * Z_Size);
    Matrix_Init(&KF->Mat.Z, KF->Z_Size, 1, (float *)KF->Data.Z);

    if (KF->U_Size != 0)
    {
				/* Initializes the external control vector */
				KF->ControlVector = (float *)User_malloc(KF->sizeof_float * U_Size);
				memset(KF->ControlVector, 0, KF->sizeof_float * U_Size);

        /* Initializes the control vector */ 
        KF->Data.U = (float *)User_malloc(KF->sizeof_float * U_Size);
        memset(KF->Data.U, 0, KF->sizeof_float * U_Size);
        Matrix_Init(&KF->Mat.U, KF->U_Size, 1, (float *)KF->Data.U);

        /* Initializes the control Matrix */  
        KF->Data.B = (float *)User_malloc(KF->sizeof_float * Xhat_Size * U_Size);
        memset(KF->Data.B, 0, KF->sizeof_float * Xhat_Size * U_Size);
        Matrix_Init(&KF->Mat.B, KF->Xhat_Size, KF->U_Size, (float *)KF->Data.B);
    }

    /* Initializes the state transition Matrix */ 
    KF->Data.A = (float *)User_malloc(KF->sizeof_float * Xhat_Size * Xhat_Size);
    memset(KF->Data.A, 0, KF->sizeof_float * Xhat_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.A, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.A);

    KF->Data.AT = (float *)User_malloc(KF->sizeof_float * Xhat_Size * Xhat_Size);
    memset(KF->Data.AT, 0, KF->sizeof_float * Xhat_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.AT, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.AT);

    /* Initializes the measurement Matrix */ 
    KF->Data.H = (float *)User_malloc(KF->sizeof_float * Z_Size * Xhat_Size);
    memset(KF->Data.H, 0, KF->sizeof_float * Z_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.H, KF->Z_Size, KF->Xhat_Size, (float *)KF->Data.H);

    KF->Data.HT = (float *)User_malloc(KF->sizeof_float * Xhat_Size * Z_Size);
    memset(KF->Data.HT, 0, KF->sizeof_float * Xhat_Size * Z_Size);
    Matrix_Init(&KF->Mat.HT, KF->Xhat_Size, KF->Z_Size, (float *)KF->Data.HT);

    /* Initializes the posteriori covariance Matrix */
    KF->Data.P = (float *)User_malloc(KF->sizeof_float * Xhat_Size * Xhat_Size);
    memset(KF->Data.P, 0, KF->sizeof_float * Xhat_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.P, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.P);

    /* Initializes the priori covariance Matrix */
    KF->Data.Pminus = (float *)User_malloc(KF->sizeof_float * Xhat_Size * Xhat_Size);
    memset(KF->Data.Pminus, 0, KF->sizeof_float * Xhat_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.Pminus, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.Pminus);

    /* Initializes the process noise covariance Matrix */  
    KF->Data.Q = (float *)User_malloc(KF->sizeof_float * Xhat_Size * Xhat_Size);
    memset(KF->Data.Q, 0, KF->sizeof_float * Xhat_Size * Xhat_Size);
    Matrix_Init(&KF->Mat.Q, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.Q);

    /* Initializes the measurement noise covariance Matrix */
    KF->Data.R = (float *)User_malloc(KF->sizeof_float * Z_Size * Z_Size);
    memset(KF->Data.R, 0, KF->sizeof_float * Z_Size * Z_Size);
    Matrix_Init(&KF->Mat.R, KF->Z_Size, KF->Z_Size, (float *)KF->Data.R);

    /* Initializes the kalman gain */  
    KF->Data.K = (float *)User_malloc(KF->sizeof_float * Xhat_Size * Z_Size);
    memset(KF->Data.K, 0, KF->sizeof_float * Xhat_Size * Z_Size);
    Matrix_Init(&KF->Mat.K, KF->Xhat_Size, KF->Z_Size, (float *)KF->Data.K);

    /* Initializes the K_denominator (K_denominator = H Pminus HT + R) */  
    KF->Data.K_denominator = (float *)User_malloc(KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    memset(KF->Data.K_denominator, 0, KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    Matrix_Init(&KF->Mat.K_denominator, KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.K_denominator);

    /* Initializes the calculate cache Matrix */
    KF->Data.Cache_Matrix[0] = (float *)User_malloc(KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    memset(KF->Data.Cache_Matrix[0],0,KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    Matrix_Init(&KF->Mat.Cache_Matrix[0], KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.Cache_Matrix[0]);

    KF->Data.Cache_Matrix[1] = (float *)User_malloc(KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    memset(KF->Data.Cache_Matrix[1],0,KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
    Matrix_Init(&KF->Mat.Cache_Matrix[1], KF->Xhat_Size, KF->Xhat_Size, (float *)KF->Data.Cache_Matrix[1]);

    /* Initializes the calculate cache vector */
    KF->Data.Cache_Vector[0] = (float *)User_malloc(KF->sizeof_float * KF->Xhat_Size);
    memset(KF->Data.Cache_Vector[0],0,KF->sizeof_float * KF->Xhat_Size);
    Matrix_Init(&KF->Mat.Cache_Vector[0], KF->Xhat_Size, 1, (float *)KF->Data.Cache_Vector[0]);

    KF->Data.Cache_Vector[1] = (float *)User_malloc(KF->sizeof_float * KF->Xhat_Size);
    memset(KF->Data.Cache_Vector[1],0,KF->sizeof_float * KF->Xhat_Size);
    Matrix_Init(&KF->Mat.Cache_Vector[1], KF->Xhat_Size, 1, (float *)KF->Data.Cache_Vector[1]);

    /* Initializes the filter output */
    KF->Output = (float *)User_malloc(KF->sizeof_float * Xhat_Size);
    memset(KF->Output, 0, KF->sizeof_float * Xhat_Size);
}

static void Kalman_Filter_Measurement_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* update the measuerment vector from the external measuerment vector */
    memcpy(KF->Data.Z, KF->MeasuredVector, KF->sizeof_float * KF->Z_Size);

    /* clear the external measuerment vector */
    memset(KF->MeasuredVector, 0, KF->sizeof_float * KF->Z_Size);

    if(KF->U_Size > 0)
    {
      /* update the control vector from the external control vector */
      memcpy(KF->Data.U, KF->ControlVector, KF->sizeof_float * KF->U_Size);
    }
}

static void Kalman_Filter_Xhatminus_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* skip this step,replaced with user function */
    if(KF->SkipStep1 == 1)
    {
      return;
    }

    if(KF->U_Size > 0)
    {
        /* Cache_Vector[0] = A Xhat(k-1) */ 
        KF->Mat.Cache_Vector[0].numRows = KF->Xhat_Size;
        KF->Mat.Cache_Vector[0].numCols = 1;
        KF->MatStatus = Matrix_Multiply(&KF->Mat.A, &KF->Mat.Xhat, &KF->Mat.Cache_Vector[0]);   

        /* Cache_Vector[1] = B u(k-1) */ 
        KF->Mat.Cache_Vector[0].numRows = KF->Xhat_Size;
        KF->Mat.Cache_Vector[0].numCols = 1;
        KF->MatStatus = Matrix_Multiply(&KF->Mat.B, &KF->Mat.U, &KF->Mat.Cache_Vector[1]);    

        /* Xhatminus = A Xhat(k-1) + B u(k-1) */
        KF->MatStatus = Matrix_Add(&KF->Mat.Cache_Vector[0], &KF->Mat.Cache_Vector[1], &KF->Mat.Xhatminus);   
    }
    /* lack of control vector */
    else
    {
        /* Xhatminus = A Xhat(k-1) */
        KF->MatStatus = Matrix_Multiply(&KF->Mat.A, &KF->Mat.Xhat, &KF->Mat.Xhatminus);   
    }
}


static void Kalman_Filter_Pminus_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* skip this step,replaced with user function */
    if(KF->SkipStep2 == 1)
    {
      return;
    }

    /* AT */
    KF->MatStatus = Matrix_Transpose(&KF->Mat.A, &KF->Mat.AT); 

    /* Pminus = A P(k-1) */ 
    KF->MatStatus = Matrix_Multiply(&KF->Mat.A, &KF->Mat.P, &KF->Mat.Pminus); 

    /* Cache_Matrix[0] = A P(k-1) AT */ 
    KF->Mat.Cache_Matrix[0].numRows = KF->Mat.Pminus.numRows;
    KF->Mat.Cache_Matrix[0].numCols = KF->Mat.AT.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Pminus, &KF->Mat.AT, &KF->Mat.Cache_Matrix[0]); 

    /* Pminus = A P(k-1) AT + Q */
    KF->MatStatus = Matrix_Add(&KF->Mat.Cache_Matrix[0], &KF->Mat.Q, &KF->Mat.Pminus);  
}


static void Kalman_Filter_K_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* skip this step,replaced with user function */
    if(KF->SkipStep3 == 1)
    {
      return;
    }

    /* HT */
    KF->MatStatus = Matrix_Transpose(&KF->Mat.H, &KF->Mat.HT); 

    /* Cache_Matrix[0] = H·Pminus */
    KF->Mat.Cache_Matrix[0].numRows = KF->Mat.H.numRows;
    KF->Mat.Cache_Matrix[0].numCols = KF->Mat.Pminus.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.H, &KF->Mat.Pminus, &KF->Mat.Cache_Matrix[0]); 

    /* Cache_Matrix[1] = H·Pminus·HT */
    KF->Mat.Cache_Matrix[1].numRows = KF->Mat.Cache_Matrix[0].numRows;
    KF->Mat.Cache_Matrix[1].numCols = KF->Mat.HT.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.HT, &KF->Mat.Cache_Matrix[1]);  

    /* K_denominator = H·Pminus·HT + R */
    KF->Mat.K_denominator.numRows = KF->Mat.R.numRows;
    KF->Mat.K_denominator.numCols = KF->Mat.R.numCols;
    KF->MatStatus = Matrix_Add(&KF->Mat.Cache_Matrix[1], &KF->Mat.R, &KF->Mat.K_denominator); 

    /* Cache_Matrix[1] = inverse(H·Pminus·HT + R) */
    KF->MatStatus = Matrix_Inverse(&KF->Mat.K_denominator, &KF->Mat.Cache_Matrix[1]);

    /* Cache_Matrix[0] = Pminus·HT */
    KF->Mat.Cache_Matrix[0].numRows = KF->Mat.Pminus.numRows;
    KF->Mat.Cache_Matrix[0].numCols = KF->Mat.HT.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Pminus, &KF->Mat.HT, &KF->Mat.Cache_Matrix[0]);

    /* K = H·Pminus / (H·Pminus·HT + R) */
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.Cache_Matrix[1], &KF->Mat.K);
}


static void Kalman_Filter_Xhat_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* skip this step,replaced with user function */
    if(KF->SkipStep4 == 1)
    {
      return;
    }

    /* Cache_Vector[0] = H Xhatminus */
    KF->Mat.Cache_Vector[0].numRows = KF->Mat.H.numRows;
    KF->Mat.Cache_Vector[0].numCols = 1;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.H, &KF->Mat.Xhatminus, &KF->Mat.Cache_Vector[0]);

    /* Cache_Vector[1] = z(k) - H·Xhatminus */
    KF->Mat.Cache_Vector[1].numRows = KF->Mat.Z.numRows;
    KF->Mat.Cache_Vector[1].numCols = 1;
    KF->MatStatus = Matrix_Subtract(&KF->Mat.Z, &KF->Mat.Cache_Vector[0], &KF->Mat.Cache_Vector[1]); 

    /* Cache_Vector[0] = K(k)·(z(k) - H·Xhatminus) */
    KF->Mat.Cache_Vector[0].numRows = KF->Mat.K.numRows;
    KF->Mat.Cache_Vector[0].numCols = 1;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.K, &KF->Mat.Cache_Vector[1], &KF->Mat.Cache_Vector[0]);

    /* Xhat = Xhatminus + K(k)·(z(k) - H·Xhatminus) */
    KF->MatStatus = Matrix_Add(&KF->Mat.Xhatminus, &KF->Mat.Cache_Vector[0], &KF->Mat.Xhat); 
}


static void Kalman_Filter_P_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* skip this step,replaced with user function */
    if(KF->SkipStep5 == 1)
    {
      return;
    }

    /* Cache_Vector[0] = K(k)·H */
    KF->Mat.Cache_Matrix[0].numRows = KF->Mat.K.numRows;
    KF->Mat.Cache_Matrix[0].numCols = KF->Mat.H.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.K, &KF->Mat.H, &KF->Mat.Cache_Matrix[0]);

    /* Cache_Vector[1] = K(k)·H·Pminus */
    KF->Mat.Cache_Matrix[1].numRows = KF->Mat.Cache_Matrix[0].numRows;
    KF->Mat.Cache_Matrix[1].numCols = KF->Mat.Pminus.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.Pminus, &KF->Mat.Cache_Matrix[1]);
		
    /* P = (I - K(k)·H)·Pminus */
    KF->MatStatus = Matrix_Subtract(&KF->Mat.Pminus, &KF->Mat.Cache_Matrix[1], &KF->Mat.P); 
}


float *Kalman_Filter_Update(KalmanFilter_Info_TypeDef *KF)
{
    /* Update the Measuerment Information */
    Kalman_Filter_Measurement_Update(KF);
    /* User Function 0 */
    if(KF->User_Function0 != NULL)
    {
      KF->User_Function0(KF);
    }

    /* Update the Priori EstiMate */
    Kalman_Filter_Xhatminus_Update(KF);
    /* User Function 1 */
    if(KF->User_Function1 != NULL)
    {
      KF->User_Function1(KF);
    }

    /* Update the Priori Covariance Matrix */
    Kalman_Filter_Pminus_Update(KF);
    /* User Function 2 */
    if(KF->User_Function2 != NULL)
    {
      KF->User_Function2(KF);
    }
    /* Update the kalman filter */
    Kalman_Filter_K_Update(KF);
    /* User Function 3 */
    if(KF->User_Function3 != NULL)
    {
      KF->User_Function3(KF);
    }

    /* Update the Posteriori EstiMate */
    Kalman_Filter_Xhat_Update(KF);
    /* User Function 4 */
    if(KF->User_Function4 != NULL)
    {
      KF->User_Function4(KF);
    }

    /* Update the Posteriori Covariance Matrix */
    Kalman_Filter_P_Update(KF);
    /* User Function 5 */
    if(KF->User_Function5 != NULL)
    {
      KF->User_Function5(KF);
    }

    /* User Function 6 */
    if(KF->User_Function6 != NULL)
    {
      KF->User_Function6(KF);
    }

    /* Update the kalman filter output */
    memcpy(KF->Output, KF->Data.Xhat, KF->sizeof_float * KF->Xhat_Size);

    return KF->Output;
}


