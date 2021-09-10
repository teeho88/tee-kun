/*
library name: 	MPU6050 6 axis module
written by: 		T.Jaber
Date Written: 	25 Mar 2019
Last Modified: 	20 April 2019 by Mohamed Yaqoob
Description: 		MPU6050 Module Basic Functions Device Driver library that use HAL libraries.
References:			
								- MPU6050 Registers map: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
								- Jeff Rowberg MPU6050 library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
								
* Copyright (C) 2019 - T. Jaber
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
	
   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.

*/

//Header files
#include "TJ_MPU6050.h"

//Library Variable
//1_ I2C Handle 
static I2C_HandleTypeDef i2cHandler;
//2_ Accel & Gyro Scaling Factor
float accelScalingFactor, gyroScalingFactor;
static uint8_t i2cBuf[2] = {0,0};
RawData_Def Accel;
RawData_Def Gyro;
int16_t accel_reg_bias[3];
int16_t Gyr_reg_bias[3];
extern int errtest;
//Fucntion Definitions
//1- i2c Handler 
void MPU6050_Init(I2C_HandleTypeDef *I2Chnd)
{
	//Copy I2C CubeMX handle to local library
	memcpy(&i2cHandler, I2Chnd, sizeof(*I2Chnd));
}

//2- i2c Read
HAL_StatusTypeDef I2C_Read(uint8_t ADDR, uint8_t *i2cBif, uint8_t NofData)
{
	uint8_t MPUADDR;
	//Need to Shift address to make it proper to i2c operation
	MPUADDR = (MPU_ADDR<<1);
	i2cBuf[0] = ADDR;
	if(HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cBuf, 1, 5)!= HAL_OK) return HAL_ERROR;
	if(HAL_I2C_Master_Receive(&i2cHandler, MPUADDR, i2cBif, NofData, 5)!= HAL_OK) return HAL_ERROR;
	return HAL_OK;
}

//3- i2c Write
HAL_StatusTypeDef I2C_Write8(uint8_t ADDR, uint8_t data)
{
	uint8_t i2cData[2];
	i2cData[0] = ADDR;
	i2cData[1] = data;
	uint8_t MPUADDR = (MPU_ADDR<<1);
	if(HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cData, 2,5)!= HAL_OK) return HAL_ERROR;
	return HAL_OK;
}

//4- MPU6050 Initialaztion Configuration 
void MPU6050_Config(MPU_ConfigTypeDef *config)
{
	uint8_t Buffer = 0;
	float g = 9.8;
	//Clock Source 
	//Reset Device
	I2C_Write8(PWR_MAGT_1_REG, 0x80);
	HAL_Delay(100);
	Buffer = config ->ClockSource & 0x07; //change the 7th bits of register
	Buffer |= (config ->Sleep_Mode_Bit << 6) &0x40; // change only the 7th bit in the register
	I2C_Write8(PWR_MAGT_1_REG, Buffer);
	HAL_Delay(100); // should wait 10ms after changeing the clock setting.
	
	//Set the Digital Low Pass Filter
	Buffer = 0;
	Buffer = config->CONFIG_DLPF & 0x07;
	I2C_Write8(CONFIG_REG, Buffer);
	
	//Select the Gyroscope Full Scale Range
	Buffer = 0;
	Buffer = (config->Gyro_Full_Scale << 3) & 0x18;
	I2C_Write8(GYRO_CONFIG_REG, Buffer);
	
	//Select the Accelerometer Full Scale Range 
	Buffer = 0; 
	Buffer = (config->Accel_Full_Scale << 3) & 0x18;
	I2C_Write8(ACCEL_CONFIG_REG, Buffer);
	//Set SRD To Default
	MPU6050_Set_SMPRT_DIV(0x04);
	
	//Accelerometer Scaling Factor, Set the Accelerometer and Gyroscope Scaling Factor
	switch (config->Accel_Full_Scale)
	{
		case AFS_SEL_2g:
			accelScalingFactor = (2.00f/32768.00f)*g;
			break;
		
		case AFS_SEL_4g:
			accelScalingFactor = (4.00f/32768.00f)*g;
				break;
		
		case AFS_SEL_8g:
			accelScalingFactor = (8.00f/32768.00f)*g;
			break;
		
		case AFS_SEL_16g:
			accelScalingFactor = (16.00f/32768.00f)*g;
			break;
		
		default:
			break;
	}
	//Gyroscope Scaling Factor 
	switch (config->Gyro_Full_Scale)
	{
		case FS_SEL_250:
			gyroScalingFactor = (250.0f/32768.0f)*3.14/180;
			break;
		
		case FS_SEL_500:
				gyroScalingFactor = (500.0f/32768.0f)*3.14/180;
				break;
		
		case FS_SEL_1000:
			gyroScalingFactor = (1000.0f/32768.0f)*3.14/180;
			break;
		
		case FS_SEL_2000:
			gyroScalingFactor = (2000.0f/32768.0f)*3.14/180;
			break;
		
		default:
			break;
	}
	
}

//5- Get Sample Rate Divider
uint8_t MPU6050_Get_SMPRT_DIV(void)
{
	uint8_t Buffer = 0;
	
	I2C_Read(SMPLRT_DIV_REG, &Buffer, 1);
	return Buffer;
}

//6- Set Sample Rate Divider
void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue)
{
	I2C_Write8(SMPLRT_DIV_REG, SMPRTvalue);
}

//7- Get External Frame Sync.
uint8_t MPU6050_Get_FSYNC(void)
{
	uint8_t Buffer = 0;
	
	I2C_Read(CONFIG_REG, &Buffer, 1);
	Buffer &= 0x38; 
	return (Buffer>>3);
}

//8- Set External Frame Sync. 
void MPU6050_Set_FSYNC(enum EXT_SYNC_SET_ENUM ext_Sync)
{
	uint8_t Buffer = 0;
	I2C_Read(CONFIG_REG, &Buffer,1);
	Buffer &= ~0x38;
	
	Buffer |= (ext_Sync <<3); 
	I2C_Write8(CONFIG_REG, Buffer);
	
}

//9- Get Accel Raw Data
HAL_StatusTypeDef ReadI2C_MPU(MPU_ConfigTypeDef *MpuConfig)
{
	uint8_t AcceArr[6];
	uint8_t GyroArr[6];
	if(I2C_Read(ACCEL_XOUT_H_REG, AcceArr,6)== HAL_OK && I2C_Read(GYRO_XOUT_H_REG, GyroArr,6) == HAL_OK)
	{
		//Accel Raw Data
		Accel.x = (int16_t)((AcceArr[0]<<8) | AcceArr[1]);
		Accel.y = (int16_t)((AcceArr[2]<<8) | AcceArr[3]);
		Accel.z = (int16_t)((AcceArr[4]<<8) | AcceArr[5]);
		//Gyro Raw Data
		Gyro.x = (int16_t)((GyroArr[0]<<8) | GyroArr[1]);
		Gyro.y = (int16_t)((GyroArr[2]<<8) | GyroArr[3]);
		Gyro.z = (int16_t)((GyroArr[4]<<8) | GyroArr[5]);
	}
	else
	{
		HAL_I2C_Init(&i2cHandler);
		MPU6050_Config(MpuConfig);
		errtest++;
		return HAL_ERROR;
	}
	return HAL_OK;
}

//10- Calibrate MPU6050
HAL_StatusTypeDef CalibrateMPU6050(MPU_ConfigTypeDef *MpuConfig)
{
	int32_t at[3] = {0}, wt[3] = {0};
	uint8_t dataA[6];
	uint8_t dataG[6];
	uint16_t count = 0;
	for(int i = 0; i<1000; i++)
	{
		if(ReadI2C_MPU(MpuConfig) == HAL_OK)
		{
			at[0] += Accel.x;
			at[1] += Accel.y;
			at[2] += (Accel.z - 16384);
			wt[0] += Gyro.x;
			wt[1] += Gyro.y;
			wt[2] += Gyro.z;
			count++;
		}
	}
	if(count < 700) return HAL_ERROR; 
	
	// calculate Gyr offset
	for(int i = 0; i < 3; i++){
		Gyr_reg_bias[i] = -((wt[i]/count)>>2);
		dataG[2*i] = (Gyr_reg_bias[i]>>8)&0xff;
		I2C_Write8(0x13 + 2*i, dataG[2*i]);
		dataG[2*i + 1] = Gyr_reg_bias[i]&0xff;
		I2C_Write8(0x14 + 2*i, dataG[2*i + 1]);
	}
	
	// calculate Acc offset
	uint16_t mask = 0x0001;
  uint8_t mask_bit[3] = {0, 0, 0};
	uint8_t AcceArr[6];
	I2C_Read(0x06, AcceArr,6);	
	for(int i=0; i<3; i++) {
		accel_reg_bias[i] = (int16_t)(AcceArr[2*i]<<8) | AcceArr[2*i+1];
		if(accel_reg_bias[i]&mask) mask_bit[i] = 0x01;
		accel_reg_bias[i] -= (at[i]/count)>>3;
		dataA[2*i] = (accel_reg_bias[i] >> 8) & 0xff;
		I2C_Write8(0x06 + 2*i, dataA[2*i]);
		dataA[2*i + 1] = accel_reg_bias[i] & 0xff;
		dataA[2*i + 1] = dataA[2*i + 1]|mask_bit[i];
		I2C_Write8(0x07 + 2*i, dataA[2*i + 1]);
	}
	return HAL_OK;
}


