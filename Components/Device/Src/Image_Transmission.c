/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Image_Transmission.c
  * @brief          : Image_Transmission_Info interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/4/20
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Image_Transmission.h"
#include "CRC.h"
#include "usart.h"

__attribute__((section (".AXI_SRAM"))) uint8_t Image_Trans_MultiRx_Buff[2][39];

Image_Transmission_Info_TypeDef Image_Transmission_Info;
VT13_Info_TypeDef VT13_Info;

static int16_t bit8TObit16(uint8_t change_info[2]);

void Image_Transmission_Info_Update(uint8_t *Buff){

	Image_Transmission_Info.Index = 0;
	Image_Transmission_Info.DataLength = 0;
	/*Check the header frame */
 
	if(Buff[0] == 0xA5){
	
	  if(Verify_CRC8_Check_Sum(&Buff[0],5) == true){
		
		  Image_Transmission_Info.DataLength = (uint16_t)(Buff[Image_Transmission_Info.Index+2]<<8 | Buff[Image_Transmission_Info.Index+1]) + FrameHeader_Length + CMDID_Length + CRC16_Length;
		 
			if(Verify_CRC16_Check_Sum(&Buff[Image_Transmission_Info.Index],Image_Transmission_Info.DataLength) == true){
				
			 #ifdef CUSTOM_ROBOT_DATA_ID
				
				if(Image_Transmission_Info.DataLength == 39){ 
					 for(uint8_t i = 0; i< 30; i++){
						 Image_Transmission_Info.custom_robot_data.data[i] = Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + i];
					 }			
				}
			 #endif
			
			 #ifdef REMOTE_CONTROL_ID
				
				if(Image_Transmission_Info.DataLength == 21){					 
					 Image_Transmission_Info.remote_control.mouse_x = bit8TObit16(&Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length]);
					 Image_Transmission_Info.remote_control.mouse_y = bit8TObit16(&Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + 2]);
					 Image_Transmission_Info.remote_control.mouse_z = bit8TObit16(&Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + 4]);
           Image_Transmission_Info.remote_control.left_button_down  = Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + 6];
				   Image_Transmission_Info.remote_control.right_button_down = Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + 7];
					 Image_Transmission_Info.remote_control.Key.keyboard_value = bit8TObit16(&Buff[Image_Transmission_Info.Index + FrameHeader_Length + CMDID_Length + 8]);
				}
			 #endif
			}
		}
		
	}else if(Buff[0] == 0xA9){
	
	     if(Buff[1] == 0x53){
			 
				  VT13_Info_Update(Buff,&VT13_Info);
			 
			 }
		}
}

void VT13_Info_Update(uint8_t *Buff ,VT13_Info_TypeDef *VT13_Info){
	
	   if(Verify_CRC16_Check_Sum(&Buff[0],21) == true){
		 
		    VT13_Info->RC.Channel[0] = ( (Buff[2]) | (Buff[3] << 8) )& 0x07FF;                           
        VT13_Info->RC.Channel[1] = ( (Buff[3] >> 3) | (Buff[4] << 5 ) ) & 0x07FF;                            
        VT13_Info->RC.Channel[2] = ( (Buff[4] >> 6) | (Buff[5] << 2 ) | (Buff[6] << 10) ) & 0x07FF;      
        VT13_Info->RC.Channel[3] = ( (Buff[6] >> 1) | (Buff[7] << 7 ) ) & 0x07FF;                         
			  VT13_Info->RC.Switch     = (  Buff[7] >> 4 ) & 0x03;
			  VT13_Info->RC.Stop       = (  Buff[7] >> 6 ) & 0x01;
			  VT13_Info->RC.Left       = (  Buff[7] >> 7 ) & 0x01;
			  VT13_Info->RC.Right      = (  Buff[8]  ) & 0x01;
			  VT13_Info->RC.Wheel      = ( (Buff[8] >> 1) | (Buff[9] << 7 ) ) & 0x07FF; 
			  VT13_Info->RC.Trigger    = (  Buff[9] >> 4 ) & 0x01;
			 
			  VT13_Info->RC.Channel[0] -= 1024;
		    VT13_Info->RC.Channel[1] -= 1024;
        VT13_Info->RC.Channel[2] -= 1024;
		    VT13_Info->RC.Channel[3] -= 1024;
		    VT13_Info->RC.Wheel      -= 1024;
			 
			  VT13_Info->Mouse.X = (Buff[10]  | (Buff[11] << 8));
			  VT13_Info->Mouse.Y = (Buff[12]  | (Buff[13] << 8));
			  VT13_Info->Mouse.Z = (Buff[14]  | (Buff[15] << 8));
			 
			  VT13_Info->Mouse.Press_L = (Buff[16]) & 0x03;
				VT13_Info->Mouse.Press_R = (Buff[16] >> 2) & 0x03;
				VT13_Info->Mouse.Press_M = (Buff[16] >> 4) & 0x03;

        VT13_Info->Key.V = (Buff[17] | (Buff[18] << 8));
		 }
		 
}


void Robot_Data_to_Custom_(uint8_t *Data){

    for(uint8_t i = 0; i<30; i++){
			
		 Image_Transmission_Info.robot_custom_data.data[i] =  Data[i];
		
		}
    
		HAL_UART_Transmit_DMA(&huart1,Image_Transmission_Info.robot_custom_data.data,30);
}

/**
 * @brief  transform the bit8 to bit16
*/
static int16_t bit8TObit16(uint8_t change_info[2])
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.byte[0] = change_info[0];
  u16val.byte[1] = change_info[1];

	return u16val.bit16;
}