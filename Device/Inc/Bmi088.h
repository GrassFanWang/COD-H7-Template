
#ifndef DEVICE_BMI088_H
#define DEVICE_BMI088_H

#include "bmi088_reg.h"
#include "stdint.h"
#include "stdbool.h"

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

typedef enum
{
    BMI088_NO_ERROR                     = 0x00,
    BMI088_ACCEL_PWR_CTRL_ERROR           = 0x01,
    BMI088_ACCEL_PWR_CONF_ERROR           = 0x02,
    BMI088_ACCEL_CONF_ERROR               = 0x03,
    BMI088_ACCEL_SELF_TEST_ERROR          = 0x04,
    BMI088_ACCEL_RANGE_ERROR              = 0x05,
    BMI088_INT1_IO_CTRL_ERROR           = 0x06,
    BMI088_INT_MAP_DATA_ERROR           = 0x07,
    BMI088_GYRO_RANGE_ERROR             = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
    BMI088_GYRO_LPM1_ERROR              = 0x0A,
    BMI088_GYRO_CTRL_ERROR              = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
    BMI088_NO_SENSOR                    = 0xFF,
}BMI088_Status_e;

typedef struct
{
	int16_t Accel_X;   
	int16_t Accel_Y;   
	int16_t Accel_Z;   

	int16_t Gyro_X;   
	int16_t Gyro_Y;  
	int16_t Gyro_Z;    

	int16_t Temperature; 
	
}MPU_Info_Typedef;


typedef struct
{
    bool Offsets_Init;   

    float Accel[3];       
    float Gyro[3];       
    float Temperature;   

    MPU_Info_Typedef MPU_Info;

    float Offsets_Gyro_X;   
    float Offsets_Gyro_Y;   
    float Offsets_Gyro_Z;   

}BMI088_Info_Typedef;

extern BMI088_Info_Typedef BMI088_Info;

extern void BMI088_Init(void);

extern void BMI088_Info_Update(BMI088_Info_Typedef *BMI088_Info);


#endif