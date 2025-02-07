#include "RLS.h"


void RLS_Init(Recursive_Least_Squares_Info_TypeDef *RLS,uint8_t X_Size,uint8_t P_Size,uint8_t Y_Size){


	 RLS->sizeof_float = sizeof(float);
	
   RLS->X_Size = X_Size;
	 RLS->P_Size = P_Size;
   RLS->Y_Size = Y_Size;
   
	 RLS->Data.W = (float *)malloc(RLS->sizeof_float * X_Size);
   memset(RLS->Data.W, 0, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.W, X_Size, 1, (float *)RLS->Data.W);
	
   RLS->Data.X = (float *)malloc(RLS->sizeof_float * X_Size);
   memset(RLS->Data.X, 0, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.X, X_Size, 1, (float *)RLS->Data.X);
	
	 RLS->Data.XT = (float *)malloc(RLS->sizeof_float * X_Size);
   memset(RLS->Data.XT, 0, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.XT,1, X_Size, (float *)RLS->Data.XT);
	
	 RLS->Data.Lamda = (float *)malloc(RLS->sizeof_float);
   memset(RLS->Data.Lamda, 0, RLS->sizeof_float);
   Matrix_Init(&RLS->Mat.Lamda,1, 1, (float *)RLS->Data.Lamda);
	
	 RLS->Data.P = (float *)malloc(RLS->sizeof_float * P_Size * P_Size);
   memset(RLS->Data.P, 0, RLS->sizeof_float * P_Size * P_Size);
   Matrix_Init(&RLS->Mat.P, P_Size, P_Size, (float *)RLS->Data.P);
    	
   RLS->Data.Y = (float *)malloc(RLS->sizeof_float * Y_Size);
   memset(RLS->Data.Y, 0, RLS->sizeof_float * Y_Size);
   Matrix_Init(&RLS->Mat.Y, Y_Size, 1, (float *)RLS->Data.Y);      

   RLS->Data.U = (float *)malloc(RLS->sizeof_float * Y_Size);
   memset(RLS->Data.U, 0, RLS->sizeof_float * Y_Size);
   Matrix_Init(&RLS->Mat.U, Y_Size, 1, (float *)RLS->Data.U);   

   RLS->Data.E = (float *)malloc(RLS->sizeof_float * Y_Size);
   memset(RLS->Data.E, 0, RLS->sizeof_float * Y_Size);
   Matrix_Init(&RLS->Mat.E, Y_Size, 1, (float *)RLS->Data.E);     
	 
	 RLS->Data.K_Numerator = (float *)malloc(RLS->sizeof_float * P_Size * P_Size);
   memset(RLS->Data.K_Numerator,0,RLS->sizeof_float * P_Size * P_Size);
   Matrix_Init(&RLS->Mat.K_Numerator, P_Size, P_Size, (float *)RLS->Data.K_Numerator);
	 
	 RLS->Data.K_Denominator = (float *)malloc(RLS->sizeof_float );
   memset(RLS->Data.K_Denominator,0,RLS->sizeof_float);
   Matrix_Init(&RLS->Mat.K_Denominator, 1, 1, (float *)RLS->Data.K_Denominator);
	 
	 RLS->Data.K = (float *)malloc(RLS->sizeof_float );
   memset(RLS->Data.K,0,RLS->sizeof_float);
   Matrix_Init(&RLS->Mat.K, X_Size, 1, (float *)RLS->Data.K);
	 
	 RLS->Data.Cache_Matrix[0] = (float *)malloc(RLS->sizeof_float * X_Size * X_Size);
   memset(RLS->Data.Cache_Matrix[0],0,RLS->sizeof_float * X_Size * X_Size);
   Matrix_Init(&RLS->Mat.Cache_Matrix[0], X_Size, X_Size, (float *)RLS->Data.Cache_Matrix[0]);

	 RLS->Data.Cache_Matrix[1] = (float *)malloc(RLS->sizeof_float * X_Size * X_Size);
   memset(RLS->Data.Cache_Matrix[1],0,RLS->sizeof_float * X_Size * X_Size);
   Matrix_Init(&RLS->Mat.Cache_Matrix[1], X_Size, X_Size, (float *)RLS->Data.Cache_Matrix[1]);
	 
	 RLS->Data.Output = (float *)malloc(RLS->sizeof_float * X_Size);
   memset(RLS->Data.Output, 0, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.Output, RLS->X_Size, 1, (float *)RLS->Data.Output);
}

void RLS_Update(Recursive_Least_Squares_Info_TypeDef *RLS){
 
    
  //E(n) = Y(n) - U(n)
	RLS->MatStatus = Matrix_Subtract(&RLS->Mat.Y,&RLS->Mat.U,&RLS->Mat.E);

  //P(n-1)*X(n) 
	RLS->Mat.K_Numerator.numRows = RLS->Mat.P.numRows;
  RLS->Mat.K_Numerator.numCols = RLS->Mat.X.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.P,& RLS->Mat.X,&RLS->Mat.K_Numerator);
	
	//XT(n) 
  RLS->MatStatus = Matrix_Transpose(&RLS->Mat.X,&RLS->Mat.XT);	
	
	//XT(n)*P(n-1)
	RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.XT.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.P.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.XT,& RLS->Mat.P,&RLS->Mat.Cache_Matrix[0]);
	
	
	//XT(n)*P(n-1)*X(n)
  RLS->Mat.Cache_Matrix[1].numRows = 1;
  RLS->Mat.Cache_Matrix[1].numCols = 1;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.Cache_Matrix[0],& RLS->Mat.X,&RLS->Mat.Cache_Matrix[1]);
	
	//lamda + XT(n)*P(n-1)*X(n)
	RLS->Mat.Cache_Matrix[0].numRows = 1;
  RLS->Mat.Cache_Matrix[0].numCols = 1;
	RLS->MatStatus = Matrix_Add(&RLS->Mat.Lamda,&RLS->Mat.Cache_Matrix[1],&RLS->Mat.Cache_Matrix[0]);

  //1/(lamda + XT(n)*P(n-1)*X(n))
  RLS->MatStatus = Matrix_Inverse(&RLS->Mat.Cache_Matrix[0],&RLS->Mat.K_Denominator);


 //K = P(n-1)*X(n) / lamda + XT(n)*P(n-1)*X(n)
	RLS->Mat.K.numRows = RLS->Mat.K_Numerator.numRows;
  RLS->Mat.K.numCols = RLS->Mat.K_Denominator.numCols;;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K_Numerator,& RLS->Mat.K_Denominator,&RLS->Mat.K);
 

// K * XT(n) 	
  RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.K.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.XT.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K,& RLS->Mat.XT,&RLS->Mat.Cache_Matrix[0]);

// K * XT(n) * P(n-1)
  RLS->Mat.Cache_Matrix[1].numRows = RLS->Mat.Cache_Matrix[0].numRows;
  RLS->Mat.Cache_Matrix[1].numCols = RLS->Mat.P.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.Cache_Matrix[0],&RLS->Mat.P,&RLS->Mat.Cache_Matrix[1]);

// P(n-1) -  K * XT(n) * P(n-1)

  RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.P.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.P.numCols;
	RLS->MatStatus = Matrix_Subtract(&RLS->Mat.P,&RLS->Mat.Cache_Matrix[1],&RLS->Mat.Cache_Matrix[0]);

// 1/lamda
  RLS->Mat.Cache_Matrix[1].numRows = 1;
  RLS->Mat.Cache_Matrix[1].numCols = 1;
	RLS->Data.Cache_Matrix[1][0] = 1/RLS->Data.Lamda[0];

//P(n) = 1/lamda *  (P(n-1) -  K * XT(n) * P(n-1))
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.Cache_Matrix[1],&RLS->Mat.Cache_Matrix[0],&RLS->Mat.P);

//W(n) = W(n-1) + K*E(n)

  RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.K.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.E.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K,&RLS->Mat.E,&RLS->Mat.Cache_Matrix[0]);

  RLS->MatStatus = Matrix_Add(&RLS->Mat.W,&RLS->Mat.Cache_Matrix[0],&RLS->Mat.Output);

  RLS->Data.W[0] = RLS->Data.Output[0];
  RLS->Data.W[1] = RLS->Data.Output[1];


}
	






