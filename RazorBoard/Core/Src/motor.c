/*
 * motor.c
 *
 *  Created on: 1 Jul 2021
 *      Author: Carl Wallmark
 */

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include <stdlib.h>
#include "uart.h"
#include "motor.h"

extern uint8_t State;
uint32_t move_timer;

#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define AVOID_OBSTACLE 5
#define FAIL 6
#define BRAKE 7
#define HARDBRAKE 8

#define SECURITY_FAIL 0
#define SECURITY_OK 1
#define SECURITY_NOSIGNAL 2
#define SECURITY_LEFT 3
#define SECURITY_RIGHT 4
#define SECURITY_BUMPER 5
#define SECURITY_IMU_FAIL 6
#define SECURITY_OUTSIDE 7
#define SECURITY_MOVEMENT 8
#define SECURITY_BACKWARD_OUTSIDE 9
#define SECURITY_STOP 10

extern uint8_t CheckSecurity(void);
extern void BLDC_send(char *cmd);
extern void delay(uint32_t time_ms);
extern uint8_t cutterStatus;
extern uint32_t rnd(uint32_t maxValue);
extern void CheckChassi(void);
extern void getIMUOrientation(void);

extern uint8_t MasterSwitch;
extern uint8_t Docked;

uint16_t M1speed = 0;
uint16_t M2speed = 0;

uint8_t convToPercent(uint16_t PWMspeed) {

	uint16_t s;

	s = round( PWMspeed / (3359 / 100));
	if (s < 0) s = 0;
	if (s > 100) s = 100;

	return s;

}
void BLDC_Motor_Forward_with_Time(uint16_t minSpeed, uint16_t maxSpeed, sram_settings settings, mpu6050 mpu, uint32_t time_ms) {

	if (MasterSwitch == 0 || Docked == 1) return;

	State = FORWARD;

	uint32_t motor_timer;
	motor_timer = HAL_GetTick();

	BLDC_send("M1R");
	BLDC_send("M2R");

	getIMUOrientation();

	for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {
		currentSpeed += 3;
		if (currentSpeed >= maxSpeed) {
			break;
		}

		uint16_t leftTilt = 0;
		uint16_t rightTilt = 0;

		if (mpu.roll < -5) {
			leftTilt = fabs(mpu.roll * settings.roll_tilt_comp);
		}
		if (mpu.roll > 5) {
			rightTilt = fabs(mpu.roll * settings.roll_tilt_comp);
		}

		char cmd[16];

		M1speed = currentSpeed - round(leftTilt);
		M2speed = currentSpeed - round(rightTilt);

		sprintf(cmd, "M1S %u", convToPercent(M1speed));
		BLDC_send(cmd);

		sprintf(cmd, "M2S %u", convToPercent(M2speed));
		BLDC_send(cmd);

		if (HAL_GetTick() - motor_timer >= time_ms) {
			break;
		}
		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_BUMPER:
			return;
		}

	}
	while (HAL_GetTick() - motor_timer < time_ms) {
		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_BUMPER:
			return;
		}
	}

	BLDC_Motor_Stop();

}
void BLDC_Motor_Forward(uint16_t minSpeed, uint16_t maxSpeed, sram_settings settings, mpu6050 mpu) {

	if (MasterSwitch == 0 || Docked == 1) return;

	State = FORWARD;

	move_timer = HAL_GetTick();

	BLDC_send("M1R");
	BLDC_send("M2R");

	getIMUOrientation();

	for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {
		currentSpeed += 3;
		if (currentSpeed >= maxSpeed) {
			break;
		}

		uint16_t leftTilt = 0;
		uint16_t rightTilt = 0;

		if (mpu.roll < -5) {
			leftTilt = fabs(mpu.roll * settings.roll_tilt_comp);
		}
		if (mpu.roll > 5) {
			rightTilt = fabs(mpu.roll * settings.roll_tilt_comp);
		}

		char cmd[16];

		M1speed = currentSpeed - round(leftTilt);
		M2speed = currentSpeed - round(rightTilt);

		sprintf(cmd, "M1S %u", convToPercent(M1speed));
		BLDC_send(cmd);

		sprintf(cmd, "M2S %u", convToPercent(M2speed));
		BLDC_send(cmd);

		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_FAIL:
			BLDC_Motor_Stop();
			return;

		case SECURITY_BUMPER:
			return;
		}

	}

}
void BLDC_Motor_Backward(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms, sram_settings settings, mpu6050 mpu) {

	if (MasterSwitch == 0 || Docked == 1) return;

	State = BACKWARD;

	uint32_t motor_timer;
	motor_timer = HAL_GetTick();

	BLDC_send("M1F");
	BLDC_send("M2F");

	getIMUOrientation();

	for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {
		currentSpeed += 3;
		if (currentSpeed >= maxSpeed) {
			break;
		}

		uint16_t leftTilt = 0;
		uint16_t rightTilt = 0;

		if (mpu.roll < -5) {
			leftTilt = fabs(mpu.roll * settings.roll_tilt_comp);
		}
		if (mpu.roll > 5) {
			rightTilt = fabs(mpu.roll * settings.roll_tilt_comp);
		}

		char cmd[16];

		M1speed = currentSpeed - round(leftTilt);
		M2speed = currentSpeed - round(rightTilt);

		sprintf(cmd, "M1S %u", convToPercent(M1speed));
		BLDC_send(cmd);

		sprintf(cmd, "M2S %u", convToPercent(M2speed));
		BLDC_send(cmd);

		if (HAL_GetTick() - motor_timer >= time_ms * 2) {
			break;
		}
		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_BUMPER:
			return;

		case SECURITY_BACKWARD_OUTSIDE:
			return;
		}

	}
	while (HAL_GetTick() - motor_timer < time_ms * 2) {
		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_BUMPER:
			return;

		case SECURITY_BACKWARD_OUTSIDE:
			return;
		}
	}

	BLDC_Motor_Stop();

}
void BLDC_Motor_Left(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {

	if (MasterSwitch == 0 || Docked == 1) return;

	State = LEFT;

	uint32_t motor_timer;
	motor_timer = HAL_GetTick();

	BLDC_send("M1R");
	BLDC_send("M2F");

	for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {
		currentSpeed += 1;
		if (currentSpeed >= maxSpeed) {
			break;
		}

		char cmd[16];

		M1speed = currentSpeed;
		M2speed = currentSpeed;

		sprintf(cmd, "M2S %u", convToPercent(M2speed));
		BLDC_send(cmd);

		sprintf(cmd, "M1S %u", convToPercent(M1speed));
		BLDC_send(cmd);

		if (HAL_GetTick() - motor_timer >= time_ms *2) {
			break;
		}
		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_BUMPER:
			return;
		}
	}

	while (HAL_GetTick() - motor_timer < time_ms *2) {
		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_BUMPER:
			return;
		}

	}

	BLDC_Motor_Stop();

}
void BLDC_Motor_Right(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {

	if (MasterSwitch == 0 || Docked == 1) return;

	State = RIGHT;

	uint32_t motor_timer;
	motor_timer = HAL_GetTick();

	BLDC_send("M1F");
	BLDC_send("M2R");

	for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {
		currentSpeed += 1;
		if (currentSpeed >= maxSpeed) {
			break;
		}

		char cmd[16];

		M1speed = currentSpeed;
		M2speed = currentSpeed;

		sprintf(cmd, "M1S %u", convToPercent(M1speed));
		BLDC_send(cmd);

		sprintf(cmd, "M2S %u", convToPercent(M2speed));
		BLDC_send(cmd);

		if (HAL_GetTick() - motor_timer >= time_ms *2) {
			break;
		}
		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_BUMPER:
			return;
			}
	}
	while (HAL_GetTick() - motor_timer < time_ms *2) {
		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_BUMPER:
			return;
		}
	}

	BLDC_Motor_Stop();

}
void BLDC_Motor_Stop(void){

	State = STOP;

	for (uint16_t currentSpeed = round((M1speed + M2speed) / 2); currentSpeed > 0; currentSpeed--) {
		currentSpeed -= 5;
		if (currentSpeed <= 300) break;

		char cmd[16];
		sprintf(cmd, "M1S %u", convToPercent(currentSpeed));
		BLDC_send(cmd);
		sprintf(cmd, "M2S %u", convToPercent(currentSpeed));
		BLDC_send(cmd);
	}
	BLDC_send("M1S 0");
	BLDC_send("M2S 0");
	M1speed = 0;
	M2speed = 0;

}
void BLDC_Motor_Brake(void){

	State = BRAKE;

	BLDC_send("M1S 0");
	BLDC_send("M2S 0");
	M1speed = 0;
	M2speed = 0;

}

void BLDC_Cutter_ON(sram_settings settings){

	cutterStatus = 1;
	extern uint32_t Boundary_Timer;

	BLDC_send("C1 start");
	BLDC_send("C1T 5.0");

	if (rnd(100000) < 50000) {
		BLDC_send("C1F");
	}
	else BLDC_send("C1R");

	Serial_Console("Cutter Motor ON\r\n");
	add_error_event("Cutter Motor ON");

	for (uint16_t cutterSpeed = 300; cutterSpeed < settings.cutterSpeed; cutterSpeed++) {
		Boundary_Timer = HAL_GetTick();

		char cmd[16];
		sprintf(cmd, "C1S %u", convToPercent(cutterSpeed));
		BLDC_send(cmd);
	}

}
void BLDC_Cutter_OFF(void){

	if (cutterStatus == 1) {
		BLDC_send("C1 stop");
		cutterStatus = 0;
	}

}
