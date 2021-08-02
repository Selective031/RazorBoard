/*
 * mpu6050.c
 *
 *  Created on: 26 Apr 2021
 *      Author: Carl Wallmark
 */
#include "main.h"
#include "sram.h"
#include "mpu6050.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
#include <math.h>

extern I2C_HandleTypeDef* razor_hi2c;
extern uint8_t board_revision;
extern mpu6050 mpu;
extern uint8_t Initial_Start;

float pitch_limit[20] = {0};
float roll_limit[20] = {0};
uint8_t pitch_limit_idx = 0;
uint8_t roll_limit_idx = 0;
float a,b;
float move_array[20] = {1.0};
uint8_t move_index = 0;
uint8_t getYawError = 1;

void Init6050() {

	uint8_t check;
	uint8_t Data;

	HAL_I2C_Mem_Read(razor_hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 50);
	HAL_Delay(20);

	Data = 0x0;
	HAL_I2C_Mem_Write(razor_hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 50);
	HAL_Delay(20);

	Data = 0x07;
	HAL_I2C_Mem_Write(razor_hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 50);
	HAL_Delay(20);

	Data = 0x0;
	HAL_I2C_Mem_Write(razor_hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 50);
	HAL_Delay(20);

	Data = 0x0;
	HAL_I2C_Mem_Write(razor_hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 50);
	HAL_Delay(20);

}

void ProcessIMUData(sram_settings settings) {

	pitch_limit[pitch_limit_idx] = raw_pitch;
	roll_limit[roll_limit_idx] = raw_roll;

	pitch_limit_idx++;
	roll_limit_idx++;

	if (pitch_limit_idx == 20) pitch_limit_idx = 0;
	if (roll_limit_idx == 20) roll_limit_idx = 0;

	int p = 0;
	int r = 0;

	for (int x = 0; x < 20; x++) {

		p += pitch_limit[x];
		r += roll_limit[x];

	}

	move_array[move_index] = sqrtf(a + b);
	move_index++;
	if (move_index == 20) move_index = 0;
	float sum = 0;
	for (int x = 0; x < 20; x++) {
		sum += move_array[x];
	}
	mpu.movement = sum / 20;

	mpu.roll = (r / 20) + settings.roll_comp;
	mpu.pitch = (p / 20) + settings.pitch_comp;

	if (mpu.yaw > 359.9) mpu.yaw = 0;
	if (mpu.yaw < 0) mpu.yaw = 359.9;

	mpu.heading = mpu.yaw;

}

void MPU6050_Read_Accel(void) {

	uint8_t Rec_Data[6];
	int16_t Accel_X_RAW = 0;
	int16_t Accel_Y_RAW = 0;
	int16_t Accel_Z_RAW = 0;

	memset(Rec_Data, 0, sizeof(Rec_Data));

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	if (HAL_I2C_Mem_Read(razor_hi2c,MPU6050_ADDR,ACCEL_XOUT_H_REG,I2C_MEMADD_SIZE_8BIT,Rec_Data,6,100) != HAL_OK) {
		reInitIMU();
		return;
	};

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Ax = (int16_t)Accel_X_RAW/16384.0;
	Ay = (int16_t)Accel_Y_RAW/16384.0;
	Az = (int16_t)Accel_Z_RAW/16384.0;

	b = (fabs(Ax) + fabs(Ay) + fabs(Az)) * 0.02;

	float r, p;

	if (board_revision == 12) {
        p = (atan2(Ax , Ay) * 57.3) - 90;										// Ay, Az
        r = atan2((- Az) , sqrt(Ay * Ay + Ax * Ax)) * 57.3;			// Ax, Ay, Az, Az
	} else {
        r = atan2(Ay , Az) * 57.3;										// Ay, Az
        p = atan2((- Ax) , sqrt(Ax * Ay + Az * Az)) * 57.3;			// Ax, Ay, Az, Az
	}

	raw_roll = r + mpu.roll_error;
	raw_pitch = p + mpu.pitch_error;


}
void MPU6050_Read_Gyro(void) {

	uint8_t Rec_Data[6];
	int16_t Gyro_X_RAW = 0;
	int16_t Gyro_Y_RAW = 0;
	int16_t Gyro_Z_RAW = 0;

	memset(Rec_Data, 0, sizeof(Rec_Data));

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	if (HAL_I2C_Mem_Read(razor_hi2c,MPU6050_ADDR,GYRO_XOUT_H_REG,I2C_MEMADD_SIZE_8BIT,Rec_Data,6,100) != HAL_OK) {
			reInitIMU();
			return;
		};

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;

	a = (fabs(Gx) + fabs(Gy) + fabs(Gz)) * 0.02;

	float yaw = board_revision == 12
	        ? Gx
	        : Gz;

	float minLimit = board_revision == 12
	        ? 5
	        : 1;

	if (getYawError == 1) {
			mpu.yaw_error = fabs(yaw);		// Auto calibrate the Gyro error at startup
			getYawError = 0;
	}

	yaw += mpu.yaw_error;					//Cancel out error

	if (yaw < -minLimit || yaw > minLimit) {		//if not moving, do not change
		mpu.yaw = mpu.yaw - (yaw * 0.02);
	}

}
