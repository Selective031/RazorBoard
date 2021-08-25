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
extern uint8_t mag_near_bwf;
extern uint8_t highgrass_slowdown;
extern uint32_t move_timer;
extern uint32_t Boundary_Timer;

extern uint8_t cutterStatus;
extern uint8_t MasterSwitch;
extern uint8_t Docked;
extern uint8_t BLDC;

extern sram_settings settings;
extern mpu6050 mpu;

uint16_t M1speed = 0;
uint16_t M2speed = 0;

void cutterHardBreak() {
	// Cutter disc hard brake

	CUTTER_FORWARD = 3359;        // Motor will hard brake when both "pins" go HIGH
	CUTTER_BACKWARD = 3359;
	delay(3000);
	cutterOFF();
}

void cutterON(void) {

	if (BLDC == 1) {
		BLDC_Cutter_ON();
		return;
	}

	cutterStatus = 1;
	uint8_t direction = 0;
	if (rnd(100000) < 50000) {
		direction = 1;
	}

	Serial_Console("Cutter Motor ON\r\n");
	add_error_event("Cutter Motor ON");

	for (uint16_t cutterSpeed = 1000; cutterSpeed < settings.cutterSpeed; cutterSpeed++) {
		Boundary_Timer = HAL_GetTick();

		if (direction == 0) {
			CUTTER_FORWARD = cutterSpeed;
			CUTTER_BACKWARD = 0;
		} else {
			CUTTER_FORWARD = 0;
			CUTTER_BACKWARD = cutterSpeed;
		}
		delay(2);
	}
}

void cutterOFF(void) {

	if (BLDC == 1) {
		BLDC_Cutter_OFF();
		return;
	}

	cutterStatus = 0;

	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
}

void UpdateMotorSpeed() {
	/* With the use of the MPU6050, we calculate the angle we are traveling in by using the gyro
	 * A simple motor speed controller, for each degree off, we simply decrease the speed with 120.
	 * TODO - a proper PID controller
	 */

	int16_t diff;
	int8_t dir = 0;
	char cmd[16];

	extern uint8_t convToPercent(uint16_t PWMspeed);

	// Calculate the difference in bearing, 0-360 accounted for. (Circular heading)
	diff = (((((int) mpu.heading - (int) mpu.hold_heading) % 360) + 540) % 360) - 180;
	diff *= settings.steering_correction;

	if (diff < 0) {
		dir = -1;
	} else if (diff > 0) {
		dir = 1;
	} else if (diff == 0) {
		dir = 0;
	}

	// Do not change speed more than 1000
	if (diff > 1000) {
		diff = 1000;
	}
	if (diff < -1000) {
		diff = -1000;
	}

	uint16_t Speed;
	if (mag_near_bwf == 1 || highgrass_slowdown == 1) {
		Speed = settings.motorMaxSpeed;
		if (mag_near_bwf == 1) {
			Speed = round(Speed * settings.proximitySpeed);
		} else if (highgrass_slowdown == 1) {
			Speed = round(Speed * 0.78);
		}
	} else {
		Speed = settings.motorMaxSpeed;
	}

	// Target is on the Left side
	if (dir > 0) {
		int CorrectedSpeed = Speed - abs(diff);

		sprintf(cmd, "M1S %u", convToPercent(CorrectedSpeed));
		BLDC_send(cmd);

		MOTOR_LEFT_FORWARD = CorrectedSpeed;
		MOTOR_RIGHT_FORWARD = Speed;
	}
	// Target is on the Right side
	else if (dir < 0) {
		int CorrectedSpeed = Speed - abs(diff);
		sprintf(cmd, "M2S %u", convToPercent(CorrectedSpeed));
		BLDC_send(cmd);
		MOTOR_LEFT_FORWARD = Speed;
		MOTOR_RIGHT_FORWARD = CorrectedSpeed;
	}
	// Spot on! Full speed ahead Captain!
	else if (dir == 0) {
		sprintf(cmd, "M1S %u", convToPercent(Speed));
		BLDC_send(cmd);
		sprintf(cmd, "M2S %u", convToPercent(Speed));
		BLDC_send(cmd);
		MOTOR_LEFT_FORWARD = Speed;
		MOTOR_RIGHT_FORWARD = Speed;
	}
}

void MotorForward(uint16_t minSpeed, uint16_t maxSpeed) {
	if (MasterSwitch == 0 || Docked == 1) return;

	if (BLDC == 1) {
		BLDC_Motor_Forward(minSpeed, maxSpeed);
		return;
	}

	State = FORWARD;

	getIMUOrientation();

	move_timer = HAL_GetTick();

	for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {
		currentSpeed += 3;
		if (currentSpeed >= maxSpeed) {
			break;
		}

		uint16_t leftTilt = 0;
		uint16_t rightTilt = 0;

		if (mpu.roll < 0) {
			leftTilt = fabs(mpu.roll * settings.roll_tilt_comp);
		}
		if (mpu.roll > 0) {
			rightTilt = fabs(mpu.roll * settings.roll_tilt_comp);
		}

		MOTOR_LEFT_BACKWARD = 0;
		MOTOR_LEFT_FORWARD = currentSpeed - round(leftTilt);

		MOTOR_RIGHT_FORWARD = currentSpeed - round(rightTilt);
		MOTOR_RIGHT_BACKWARD = 0;

		HAL_Delay(1);

		if (CheckSecurity() == SECURITY_FAIL) {
			MotorStop();
			break;
		}
	}
}

void MotorBackward(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {
	if (MasterSwitch == 0 || Docked == 1) return;

	if (BLDC == 1) {
		BLDC_Motor_Backward(minSpeed, maxSpeed, time_ms);
		return;
	}

	MotorBackwardImpl(minSpeed, maxSpeed, time_ms, false);
}

void MotorBackwardImpl(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms, bool forced) {
	if (MasterSwitch == 0 || Docked == 1) return;

	add_error_event("MotorBackward");
	uint32_t motor_timer;
	State = BACKWARD;
	motor_timer = HAL_GetTick();

	getIMUOrientation();

	for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {
		currentSpeed += 3;
		if (currentSpeed >= maxSpeed) {
			break;
		}

		uint16_t leftTilt = 0;
		uint16_t rightTilt = 0;

		if (mpu.roll < 0) {
			leftTilt = fabs(mpu.roll * settings.roll_tilt_comp);
		}
		if (mpu.roll > 0) {
			rightTilt = fabs(mpu.roll * settings.roll_tilt_comp);
		}

		MOTOR_LEFT_BACKWARD = currentSpeed - round(rightTilt);
		MOTOR_LEFT_FORWARD = 0;

		MOTOR_RIGHT_FORWARD = 0;
		MOTOR_RIGHT_BACKWARD = currentSpeed - round(leftTilt);

		HAL_Delay(1);

		if (!forced && CheckSecurity() == SECURITY_BACKWARD_OUTSIDE) {
			MotorHardBrake();
			delay(1000);
			MotorForward(settings.motorMinSpeed, settings.motorMaxSpeed);
			delay(500);
			MotorStop();
			delay(500);
			return;
		}

		if (HAL_GetTick() - motor_timer >= time_ms) {
			break;
		}
	}
	while (HAL_GetTick() - motor_timer < time_ms) {
		if (!forced && CheckSecurity() == SECURITY_BACKWARD_OUTSIDE) {
			MotorHardBrake();
			delay(1000);
			MotorForward(settings.motorMinSpeed, settings.motorMaxSpeed);
			delay(500);
			MotorStop();
			delay(500);
			return;
		}

		delay(100);
	}
	MotorStop();
}

void MotorRight(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {
	if (MasterSwitch == 0 || Docked == 1) return;

	if (BLDC == 1) {
		BLDC_Motor_Right(minSpeed, maxSpeed, time_ms);
		return;
	}

	add_error_event("MotorRight");
	State = RIGHT;
	uint32_t motor_timer;
	motor_timer = HAL_GetTick();

	for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {
		currentSpeed += 3;
		if (currentSpeed >= maxSpeed) {
			break;
		}
		MOTOR_LEFT_BACKWARD = 0;
		MOTOR_LEFT_FORWARD = currentSpeed;

		MOTOR_RIGHT_FORWARD = 0;
		MOTOR_RIGHT_BACKWARD = currentSpeed;

		HAL_Delay(1);

		CheckSecurity();

		if (HAL_GetTick() - motor_timer >= time_ms) {
			break;
		}
	}
	while (HAL_GetTick() - motor_timer < time_ms) {
		CheckSecurity();
	}
	MotorStop();
}

void MotorLeft(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {
	if (MasterSwitch == 0 || Docked == 1) return;

	if (BLDC == 1) {
		BLDC_Motor_Left(minSpeed, maxSpeed, time_ms);
		return;
	}

	add_error_event("MotorLeft");
	State = LEFT;
	uint32_t motor_timer;
	motor_timer = HAL_GetTick();

	for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {
		currentSpeed += 3;
		if (currentSpeed >= maxSpeed) {
			break;
		}
		MOTOR_LEFT_BACKWARD = currentSpeed;
		MOTOR_LEFT_FORWARD = 0;

		MOTOR_RIGHT_FORWARD = currentSpeed;
		MOTOR_RIGHT_BACKWARD = 0;

		HAL_Delay(1);

		CheckSecurity();

		if (HAL_GetTick() - motor_timer >= time_ms) {
			break;
		}
	}
	while (HAL_GetTick() - motor_timer < time_ms) {
		CheckSecurity();
	}
	MotorStop();
}

void MotorStop(void) {

	if (BLDC == 1) {
		BLDC_Motor_Stop();
		return;
	}

	State = STOP;
	int speed = 0;

	speed = (MOTOR_LEFT_BACKWARD + MOTOR_LEFT_FORWARD + MOTOR_RIGHT_FORWARD + MOTOR_RIGHT_BACKWARD) / 2;
	speed *= 0.90;

	if (speed == 0) {
		return;
	}

	for (int x = speed; x > 1000; x--) {
		x -= 4;

		if (x < 1000) {
			break;
		}

		if (MOTOR_LEFT_BACKWARD != 0)
			MOTOR_LEFT_BACKWARD = x;
		if (MOTOR_LEFT_FORWARD != 0)
			MOTOR_LEFT_FORWARD = x;
		if (MOTOR_RIGHT_FORWARD != 0)
			MOTOR_RIGHT_FORWARD = x;
		if (MOTOR_RIGHT_BACKWARD != 0)
			MOTOR_RIGHT_BACKWARD = x;

		HAL_Delay(1);
	}

	MOTOR_LEFT_BACKWARD = 0;
	MOTOR_LEFT_FORWARD = 0;
	MOTOR_RIGHT_FORWARD = 0;
	MOTOR_RIGHT_BACKWARD = 0;
}


void MotorBrake(void) {
	if (MasterSwitch == 0 || Docked == 1) return;

	if (BLDC == 1) {
		BLDC_Motor_Brake();
		return;
	}

	State = BRAKE;

	// Brake - free wheeling
	MOTOR_LEFT_BACKWARD = 0;
	MOTOR_LEFT_FORWARD = 0;
	MOTOR_RIGHT_FORWARD = 0;
	MOTOR_RIGHT_BACKWARD = 0;
}

void MotorHardBrake(void) {
	if (MasterSwitch == 0 || Docked == 1) return;

	if (BLDC == 1) {
		BLDC_Motor_Brake();
		return;
	}

	State = HARDBRAKE;

	// Wheels will do a hard brake when both pins go HIGH.
	MOTOR_LEFT_BACKWARD = 3359;
	MOTOR_LEFT_FORWARD = 3359;
	MOTOR_RIGHT_FORWARD = 3359;
	MOTOR_RIGHT_BACKWARD = 3359;

	HAL_Delay(250);

	MotorBrake();    //Release motors

}

uint8_t convToPercent(uint16_t PWMspeed) {

	uint16_t s;

	s = round( PWMspeed / (3359 / 100));
	if (s < 0) s = 0;
	if (s > 100) s = 100;

	return s;

}
void BLDC_Motor_Forward_with_Time(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {

	if (MasterSwitch == 0 || Docked == 1) return;

	State = FORWARD;

	uint32_t motor_timer;
	motor_timer = HAL_GetTick();
	move_timer = HAL_GetTick();

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

		case SECURITY_FAIL:
			return;
		}

	}
	while (HAL_GetTick() - motor_timer < time_ms * 2) {
		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_BUMPER:
			return;

		case SECURITY_FAIL:
			return;
		}
	}

	BLDC_Motor_Stop();

}
void BLDC_Motor_Forward(uint16_t minSpeed, uint16_t maxSpeed) {

	if (MasterSwitch == 0 || Docked == 1) return;

	State = FORWARD;

	move_timer = HAL_GetTick();

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
void BLDC_Motor_Backward(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {

	if (MasterSwitch == 0 || Docked == 1) return;

	State = BACKWARD;

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
		currentSpeed += 3;
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

		if (HAL_GetTick() - motor_timer >= time_ms * 2) {
			break;
		}
		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_BUMPER:
			return;
		}
	}

	while (HAL_GetTick() - motor_timer < time_ms * 2) {
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
		currentSpeed += 3;
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

		if (HAL_GetTick() - motor_timer >= time_ms * 2) {
			break;
		}
		switch (CheckSecurity()) {

		case SECURITY_STOP:
			return;

		case SECURITY_BUMPER:
			return;
			}
	}
	while (HAL_GetTick() - motor_timer < time_ms * 2) {
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

void BLDC_Cutter_ON(void){

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
		delay(1);
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
