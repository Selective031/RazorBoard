/*
 * motor.h
 *
 *  Created on: 1 Jul 2021
 *      Author: Carl Wallmark
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "sram.h"
#include "mpu6050.h"
#include <stdbool.h>

#define CUTTER_FORWARD TIM3->CCR1
#define CUTTER_BACKWARD TIM3->CCR2
#define MOTOR_LEFT_FORWARD TIM4->CCR2
#define MOTOR_LEFT_BACKWARD TIM4->CCR1
#define MOTOR_RIGHT_FORWARD TIM4->CCR3
#define MOTOR_RIGHT_BACKWARD TIM4->CCR4


void UpdateMotorSpeed();

// Internal drivers
void cutterON(void);
void cutterOFF(void);
void MotorHardBrake(void);
void MotorBrake(void);
void MotorStop(void);
void MotorForward(uint16_t minSpeed, uint16_t maxSpeed);
void MotorBackward(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
void MotorBackwardImpl(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms, bool forced);
void MotorLeft(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
void MotorRight(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);

// BLDC - Raptor
void BLDC_Motor_Forward_with_Time(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
void BLDC_Motor_Forward(uint16_t minSpeed, uint16_t maxSpeed);
void BLDC_Motor_Backward(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
void BLDC_Motor_Left(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
void BLDC_Motor_Right(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
void BLDC_Motor_Stop();
void BLDC_Motor_Brake();
void BLDC_Cutter_ON(void);
void BLDC_Cutter_OFF(void);

#endif /* INC_MOTOR_H_ */
