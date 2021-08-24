/*
 * mpu6050.h
 *
 *  Created on: 22 Apr 2021
 *      Author: Carl Wallmark
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

float Ax, Ay, Az, Gx, Gy, Gz;
float raw_roll, raw_pitch;

typedef struct MP6050 {

	float heading;
	float yaw;
	float roll;
	float pitch;
	float hold_heading;
	float yaw_error;
	float movement;

} mpu6050;

void Init6050(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void ProcessIMUData(sram_settings settings);

#endif /* INC_MPU6050_H_ */
