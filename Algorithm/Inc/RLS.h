#ifndef RLS_H
#define RLS_H


#include "Config.h"
#include "stdint.h"
#include "arm_math.h"

#define Matrix             arm_matrix_instance_f32
#define Matrix_64          arm_matrix_instance_f64
#define Matrix_Init        arm_mat_init_f32
#define Matrix_Add         arm_mat_add_f32
#define Matrix_Subtract    arm_mat_sub_f32
#define Matrix_Multiply    arm_mat_mult_f32
#define Matrix_Transpose   arm_mat_trans_f32
#define Matrix_Inverse     arm_mat_inverse_f32
#define Matrix_Inverse_64  arm_mat_inverse_f64


typedef struct
{

  uint8_t sizeof_float;
	
	uint8_t X_Size;
	uint8_t Y_Size;
	uint8_t P_Size;
	float Lamda; //forgetting factor

 struct 
  {
    Matrix X; 
    Matrix XT; 
    Matrix Lamda; 		
    Matrix E;                 
    Matrix Z;
    Matrix K;  		
    Matrix W;         		
    Matrix P;
    Matrix Y;
    Matrix U;		
    Matrix K_Numerator;                 
    Matrix K_Denominator;    
    Matrix Cache_Matrix[2];   
    Matrix Cache_Vector[2];   
		Matrix Output;  
  }Mat;
	
	 arm_status MatStatus;
	
	 struct 
  {
    float *X;
    float	*XT;
    float *Lamda;	
    float *E;                 
    float *Z;
    float *K;		
    float *W; 
		float *Y;		
    float *U;                 
    float *P;                 
    float *K_Numerator;                 
    float *K_Denominator;    
    float *Cache_Matrix[2];   
    float *Cache_Vector[2];  
    float *Output;
 		
  }Data;

	
	
	
}Recursive_Least_Squares_Info_TypeDef;


extern Recursive_Least_Squares_Info_TypeDef Power_Control_RLS_Info;

extern void RLS_Update(Recursive_Least_Squares_Info_TypeDef *RLS);
extern void RLS_Init(Recursive_Least_Squares_Info_TypeDef *RLS,uint8_t X_Size,uint8_t P_Size,uint8_t Y_Size);




#endif