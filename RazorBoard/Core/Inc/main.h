/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

void reInitIMU(void);
uint8_t CheckSecurity(void);
void delay(uint32_t time_ms);
void CheckChassi(void);
void getIMUOrientation(void);
uint32_t rnd(uint32_t maxValue);
void delay_us(uint16_t us);
void CheckBWF(void);
void CheckBWF_Rear(void);
void CheckState(void);
void parseCommand_RPI(void);
void parseCommand_Console(void);
void perimeterTracker(void);
void ChargerConnected(void);
void ADC_Send(uint8_t channel);
int ADC_Receive();
void CollectADC(void);
void SendInfo(void);
void CheckMotorCurrent(int RAW);
void unDock(void);
void InitFIR(void);
void FIR_LEFT(void);
void FIR_RIGHT(void);
void FIR_REAR(void);
void WatchdogRefresh(void);
void WatchdogInit(void);
void (*SysMemBootJump)(void);
void BootLoaderInit(unsigned long BootLoaderStatus);
void setTime(uint8_t hour, uint8_t minute, uint8_t second);
void setDate(uint8_t year, uint8_t month, uint8_t day, uint8_t weekday);
void TimeToGoHome(void);
void CalcMagnitude(uint8_t idx, uint8_t Sensor);
void i2c_scanner(void);
void scanner(I2C_HandleTypeDef i2c_bus);
void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef* handle, uint32_t timeout);
bool wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout);
void PIDtracker(void);
void buzzer(uint8_t count, uint16_t time_between);

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

char msg[128];
char emsg[50];

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define chassiSensor_Pin GPIO_PIN_7
#define chassiSensor_GPIO_Port GPIOE
#define chassiSensor_EXTI_IRQn EXTI9_5_IRQn
#define stopButton_Pin GPIO_PIN_8
#define stopButton_GPIO_Port GPIOE
#define stopButton_EXTI_IRQn EXTI9_5_IRQn
#define buzzer_Pin GPIO_PIN_14
#define buzzer_GPIO_Port GPIOB
#define voltage_sens_Pin GPIO_PIN_8
#define voltage_sens_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
