#include "Bmi088.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_tick.h"

static void BMI088_Write_Single_Reg(uint8_t, uint8_t);

static void BMI088_Read_Single_Reg(uint8_t, uint8_t *);

static void BMI088_Read_Multi_Reg(uint8_t, uint8_t *, uint8_t);

#define BMI088_Accel_Write_Single_Reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }

#define BMI088_Accel_Read_Single_Reg(reg, data) \
    {                                           \
       BMI088_ACCEL_NS_L();                     \
       BMI088_Read_Write_Byte((reg) | 0x80);    \
       BMI088_Read_Write_Byte(0x55);            \
			 (data) = BMI088_Read_Write_Byte(0x55);   \
       BMI088_ACCEL_NS_H();                     \
    }
		
#define BMI088_Accel_Read_Multi_Reg(reg, data, len) \
    {                                               \
        BMI088_ACCEL_NS_L();                        \
        BMI088_Read_Write_Byte((reg) | 0x80);       \
        BMI088_Read_Multi_Reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                        \
    }

#define BMI088_Gyro_Write_Single_Reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }

#define BMI088_Gyro_Read_Single_Reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Read_Single_Reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }

#define BMI088_Gyro_Read_Multi_Reg(reg, data, len)   \
    {                                                \
        BMI088_GYRO_NS_L();                          \
        BMI088_Read_Multi_Reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                          \
    }

static float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;  

static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;		
		
BMI088_Info_Typedef BMI088_Info;
		
static uint8_t Accel_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    /* Turn on accelerometer */
    {BMI088_ACCEL_PWR_CTRL, BMI088_ACCEL_ENABLE_ACCEL_ON, BMI088_ACCEL_PWR_CTRL_ERROR},   

    /* Pause mode */
    {BMI088_ACCEL_PWR_CONF, BMI088_ACCEL_PWR_ACTIVE_MODE, BMI088_ACCEL_PWR_CONF_ERROR}, 

    /* ACCELeleration Configuration */
    {BMI088_ACCEL_CONF,  (BMI088_ACCEL_NORMAL| BMI088_ACCEL_1600_HZ | BMI088_ACCEL_CONF_MUST_Set), BMI088_ACCEL_CONF_ERROR}, 

    /* ACCELelerometer setting range */ 
    {BMI088_ACCEL_RANGE, BMI088_ACCEL_RANGE_6G, BMI088_ACCEL_RANGE_ERROR},  

    /* INT1 Configuration input and output pin */ 
    {BMI088_INT1_IO_CTRL, (BMI088_ACCEL_INT1_IO_ENABLE | BMI088_ACCEL_INT1_GPIO_PP | BMI088_ACCEL_INT1_GPIO_LOW), BMI088_INT1_IO_CTRL_ERROR}, 

    /* interrupt map pin */
    {BMI088_INT_MAP_DATA, BMI088_ACCEL_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}  
};		
		
static uint8_t Gyro_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    /* Angular rate and resolution */
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR}, 

    /* Data Transfer Rate and Bandwidth Settings */
    {BMI088_GYRO_BANDWIDTH, (BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set), BMI088_GYRO_BANDWIDTH_ERROR}, 

    /* Power Mode Selection Register */
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},   

    /* Data Interrupt Trigger Register */
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},   

    /* Interrupt Pin Trigger Register */
    {BMI088_GYRO_INT3_INT4_IO_CONF, (BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW), BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},  

    /* interrupt map register */
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}   
};	

static BMI088_Status_e BMI088_Accel_Init(void)
{
    
    uint8_t res = 0;

    /* check the communication ------------------------------------------------*/
    /* read the accelerator ID address */
	  BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    /* read again */
    BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* ACCELelerator software Reset ------------------------------------------------*/
    /* write 0xB6 to the register ACCEL_SOFTResET that addRess is 0x7E to reset software */
    BMI088_Accel_Write_Single_Reg(BMI088_ACCEL_SOFTRESET, BMI088_ACCEL_SOFTRESET_VALUE); 
    /* software reset waiting time, there is 80ms */
    Delay_ms(BMI088_LONG_DELAY_TIME);

    /* check the communication again ------------------------------------------------*/
    BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Accel_Read_Single_Reg(BMI088_ACCEL_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* if the value of the 0x00 register is not 0x1E,reset failed */
    if (res != BMI088_ACCEL_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* config the accelerator sensor */
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_Accel_Write_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0], Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1]); 
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* read the configuration */
        BMI088_Accel_Read_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0], res);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        
        /* check the configuration and return the specified error */
        if (res != Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1])
        {
            return (BMI088_Status_e)Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][2];
        }
    }

    /* no error */
    return BMI088_NO_ERROR;  
}

static BMI088_Status_e BMI088_Gyro_Init(void)
{
    uint8_t res = 0;

    /* check the communication ------------------------------------------------*/
    /* read the gyro ID address */
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    /* read again */
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* gyro software reset ------------------------------------------------*/
    /* write 0xB6 to the register GYRO_SOFTRESET that address is 0x14 to reset software */
    BMI088_Gyro_Write_Single_Reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    /* software reset waiting time, there is 80ms */
    Delay_ms(BMI088_LONG_DELAY_TIME);

    /* check the communication again ------------------------------------------------*/
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* if the value of the 0x00 register is not 0x0F,reset failed */
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* config the gyro sensor */
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
				
        BMI088_Gyro_Write_Single_Reg(Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][0], Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][1]);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* read the configuration */
        BMI088_Gyro_Read_Single_Reg(Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][0], res);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* check the configuration and return the specified error */
        if (res != Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][1])
        {
            return (BMI088_Status_e)Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][2];
        }
    }

    /* no error */
    return BMI088_NO_ERROR;
}

#define IMU_Calibration_ENABLE  0U

static void BMI088_Offset_Update(BMI088_Info_Typedef *BMI088_Info)
{
#if IMU_Calibration_ENABLE /* ENABLE the BMI088 Calibration */
	
	uint8_t buf[8] = {0,};

  for(uint16_t i = 0; i < 5000; i++)
  {
    /* read the accelerator multi data */
    BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
    BMI088_Info->MPU_Info.Accel_X = (int16_t)((buf[1]) << 8) | buf[0];
    BMI088_Info->MPU_Info.Accel_Y = (int16_t)((buf[3]) << 8) | buf[2];
    BMI088_Info->MPU_Info.Accel_Z = (int16_t)((buf[5]) << 8) | buf[4];

    /* read the gyro multi data */
    BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
    /* check the ID */
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)   
    {
      BMI088_Info->MPU_Info.Gyro_X = (int16_t)((buf[3]) << 8) | buf[2];
      BMI088_Info->MPU_Info.Gyro_Y = (int16_t)((buf[5]) << 8) | buf[4];
      BMI088_Info->MPU_Info.Gyro_Z = (int16_t)((buf[7]) << 8) | buf[6];

      /* update the gyro offsets */
      BMI088_Info->Offsets_Gyro_X += BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_X;
      BMI088_Info->Offsets_Gyro_Y += BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Y;
      BMI088_Info->Offsets_Gyro_Z += BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Z;
    }
    /* waiting 1ms */
    Delay_ms(1);
  }

  BMI088_Info->Offsets_Gyro_X = BMI088_Info->Offsets_Gyro_X / 5000.f;
  BMI088_Info->Offsets_Gyro_Y = BMI088_Info->Offsets_Gyro_Y / 5000.f; 
  BMI088_Info->Offsets_Gyro_Z = BMI088_Info->Offsets_Gyro_Z / 5000.f;
	
#else /* DISABLE the BMI088 Calibration */
	/* store the previous offsets */
  BMI088_Info->Offsets_Gyro_X =  0.00379787781f;
  BMI088_Info->Offsets_Gyro_Y =  -0.00349405757f;
  BMI088_Info->Offsets_Gyro_Z =  9.18259611e-05f;
#endif

  /* set the offset init flag */
  BMI088_Info->Offsets_Init = true;
}

void BMI088_Init(void)
{
  BMI088_Status_e status = BMI088_NO_ERROR;

  /* Initializes the BMI088 */
 
	do{	
    status |= BMI088_Accel_Init();
		
    status |= BMI088_Gyro_Init();

    Delay_ms(2);
	
	}while(status);
  
	BMI088_Offset_Update(&BMI088_Info);

}

void BMI088_Info_Update(BMI088_Info_Typedef *BMI088_Info)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};

    /* read the accelerator multi data */
		BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
    BMI088_Info->MPU_Info.Accel_X = (int16_t)((buf[1] << 8) | buf[0]);
    BMI088_Info->MPU_Info.Accel_Y = (int16_t)((buf[3] << 8) | buf[2]);
    BMI088_Info->MPU_Info.Accel_Z = (int16_t)((buf[5] << 8) | buf[4]);

    /* converts the accelerator data */
    BMI088_Info->Accel[0] =  BMI088_ACCEL_SEN * BMI088_Info->MPU_Info.Accel_X;
		BMI088_Info->Accel[1] =  BMI088_ACCEL_SEN * BMI088_Info->MPU_Info.Accel_Y;
		BMI088_Info->Accel[2] =  BMI088_ACCEL_SEN * BMI088_Info->MPU_Info.Accel_Z;

    /* read the temperature */
    BMI088_Accel_Read_Multi_Reg(BMI088_TEMP_M, buf, 2);
    BMI088_Info->MPU_Info.Temperature = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (BMI088_Info->MPU_Info.Temperature > 1023) BMI088_Info->MPU_Info.Temperature -= 2048;

    /* converts the temperature data */
    BMI088_Info->Temperature = BMI088_Info->MPU_Info.Temperature * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    /* read the gyro multi data */
		BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
    /* check the ID */
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)   
    {
      BMI088_Info->MPU_Info.Gyro_X = (int16_t)((buf[3] << 8) | buf[2]);
      BMI088_Info->MPU_Info.Gyro_Y = (int16_t)((buf[5] << 8) | buf[4]);
      BMI088_Info->MPU_Info.Gyro_Z = (int16_t)((buf[7] << 8) | buf[6]);
    }

    /* converts the gyro data */
		BMI088_Info->Gyro[0] =   BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_X - BMI088_Info->Offsets_Gyro_X;
	  BMI088_Info->Gyro[1] =   BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Y - BMI088_Info->Offsets_Gyro_Y;
	  BMI088_Info->Gyro[2] =   BMI088_GYRO_SEN * BMI088_Info->MPU_Info.Gyro_Z - BMI088_Info->Offsets_Gyro_Z;
}


static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data)
{
    BMI088_Read_Write_Byte(reg);  

    BMI088_Read_Write_Byte(data);
}


static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t *return_data)
{

    BMI088_Read_Write_Byte(reg | 0x80); 

    *return_data = BMI088_Read_Write_Byte(0x55);
}

static void BMI088_Read_Multi_Reg(uint8_t reg, uint8_t *buf, uint8_t len)
{

  	BMI088_Read_Write_Byte(reg | 0x80);

    while (len != 0)
    {
     
        *buf = BMI088_Read_Write_Byte(0x55); 
        buf++;
        len--;
   
		}
}