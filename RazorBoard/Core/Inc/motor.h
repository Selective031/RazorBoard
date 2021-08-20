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

void BLDC_Motor_Forward(uint16_t minSpeed, uint16_t maxSpeed, sram_settings settings, mpu6050 mpu);
void BLDC_Motor_Backward(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
void BLDC_Motor_Left(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
void BLDC_Motor_Right(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
void BLDC_Motor_Stop();
void BLDC_Motor_Brake();
void BLDC_Cutter_ON(sram_settings settings);
void BLDC_Cutter_OFF(void);

#endif /* INC_MOTOR_H_ */
