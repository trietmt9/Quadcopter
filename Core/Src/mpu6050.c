#include <mpu6050.h>

extern I2C_HandleTypeDef hi2c1;


void MPU6050_Init()
{
    uint8_t check,Data; 

    HAL_I2C_Mem_Read(&hi2c1, (MPU6050_Address<<1), WHO_AM_I, 1, &check, 1,100);
    if(check == MPU6050_Address)
    {
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_Address<<1), PWR_MGMT_1, 1, &Data, 1, 100);

        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_Address<<1), SMPRT_DIV, 1, &Data, 1, 100);

        Data = 0x10;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_Address<<1), ACCEL_CONFIG, 1,&Data, 1, 100);

        Data = 0x08;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_Address<<1), GYRO_CONFIG, 1,&Data, 1, 100);
    }
}


void MPU6050_Read(mpu6050_t *Data)
{
    uint8_t MPU6050_Data[14];
    HAL_I2C_Mem_Read_DMA(&hi2c1, (MPU6050_Address<<1), ACCEL_XOUT, 1, &MPU6050_Data, 6);
    Data->Ax_RAW =(int16_t) (MPU6050_Data[0] <<8 | MPU6050_Data[1]);
    Data->Ay_RAW =(int16_t) (MPU6050_Data[2] <<8 | MPU6050_Data[3]);
    Data->Az_RAW =(int16_t) (MPU6050_Data[4] <<8 | MPU6050_Data[5]);
 
    Data->Gx_RAW =(int16_t) (MPU6050_Data[8] << 8 | MPU6050_Data[9]);
    Data->Gy_RAW =(int16_t) (MPU6050_Data[10] << 8 | MPU6050_Data[11]);
    Data->Gz_RAW =(int16_t) (MPU6050_Data[12] << 8 | MPU6050_Data[13]);
}

void MPU6050_Calculate(mpu6050_t *Data)
{
    Data->Gx = Data->Gx_RAW/65.5;
    Data->Gy = Data->Gy_RAW/65.5;
    Data->Gz = Data->Gz_RAW/65.5;

    Data->Ax = Data->Ax_RAW/4096.0;
    Data->Ay = Data->Ay_RAW/4096.0;
    Data->Az = Data->Az_RAW/4096.0;

    Data->Roll = atan(Data->Ax/(sqrt(pow(Data->Ay,2)+pow(Data->Az,2))))*RAD2DEG;
    Data->Pitch = atan((-1*Data->Ay)/(sqrt( pow(Data->Ax,2) + pow(Data->Az,2) )))*RAD2DEG;
}