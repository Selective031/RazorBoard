/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define ARM_MATH_CM4

#include "stdio.h"
#include "string.h"
#include "math.h"
#include <stdlib.h>
#include <time.h>
#include <arm_math.h>
#include <fir_filter.h>
#include <signature.h>
#include <ctype.h>
#include <help.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

IWDG_HandleTypeDef hiwdg;

#define ADS1115_ADDRESS 0x48

#define M1_addr 0xC1		//M1 Current Sensor
#define V1_addr 0xD1		//Voltage Sensor
#define M2_addr 0xE1		//M2 Current Sensor
#define C1_addr 0xF1		//C1 Current Sensor

int M1_Value = 0;
int M2_Value = 0;
int C1_Value = 0;
int V1_Value = 0;

uint8_t Channel = M1_addr;
uint8_t Channel_Status = 0;

#define TRUE 1
#define FALSE 0

#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define AVOID_OBSTACLE 5
#define FAIL 6
#define BRAKE 7
#define HARDBRAKE 8

#define NOSIGNAL 0
#define INSIDE 1
#define OUTSIDE 2

#define SECURITY_FAIL 0
#define SECURITY_OK 1
#define SECURITY_NOSIGNAL 2
#define SECURITY_LEFT 3
#define SECURITY_RIGHT 4
#define SECURITY_BUMBER 5

#define INITIAL_MAX_THRESHOLD 10000

#define MotorMaxSpeed 3360 -1		// 3359 is max PWM for 25Khz at 84 MHz with APB1 Clock
#define MotorMinSpeed 2000			// Minimum speed to start with

#define CUTTER_MAX_SPEED 2650		// max is 3359 at 25Khz

#define PI_BFR_SIZE 64				// Buffer size for RPi
#define CONSOLE_BFR_SIZE 64			// Buffer size for Serial Console

#define MPU6050_ADDR 0xD0			// Registers for MPU 6050
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define Voltage_Fully_Charged 25000	// When to consider battery fully charged, if using lithium, don't fully charge, the battery will last longer.
#define Voltage_Limit_LOW 22000		// When battery is "empty"

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;
float raw_roll, raw_pitch;
float yaw = 0;

double Tick1 = 0;
double Tick2 = 0;
float kp = 0.12;
float ki = 0.0;
float kd = 0.01;
double error = 0;
double lastError = 0;
double cumError = 0;
double elapsedTime = 0;
double rateError = 0;
double previousTime = 0;
uint8_t perimeterTracking = 0;
uint8_t perimeterTrackingActive = 0;

uint8_t Initial_Start = 0;
uint16_t Start_Threshold = 0;
uint8_t Outside_Threshold = 8;			// How long we can be outside before disabling the system, in seconds
float Signal_Integrity_IN = 0.80;		// Classified as IN
float Signal_Integrity_OUT = -0.80;		// Classified as OUT

int bwf1_inside = 0;				// Some stats, how many messages we can detect per second
int bwf2_inside = 0;
int bwf1_outside = 0;
int bwf2_outside = 0;

uint32_t Boundary_Timer;			// Keep track of time between signals.
uint8_t Boundary_Timeout = 6;		// in seconds, mower will stop if no signal has been detected for X seconds.

float M1_amp,M2_amp,C1_amp = 0.0f;	// Values for AMP per motor
float M1_error = 0.0f;				// Auto calibration value stored here.
float M2_error = 0.0f;				// Auto calibration value stored here.
float C1_error = 0.0f;				// Auto calibration value stored here.
float Voltage = 25200.00;			// Max voltage, only to feed the system something while starting up.
float Motor_Limit = 3.0;			// Limit for power spikes in motors
float Cutter_Limit = 1.0;			// Limit in AMP for the cutter disk, when reached the system will disable until reboot.

uint8_t BWF1_Status = 0;
uint8_t BWF2_Status = 0;
uint8_t State = STOP;
uint8_t LastState = STOP;
uint8_t cutterStatus = 0;
uint8_t AVOID_LEFT = 0;
uint8_t AVOID_RIGHT = 0;
uint8_t AVOID_FRONT = 0;

uint32_t ADC_timer = 0;
uint32_t IMU_timer = 0;
uint32_t OUTSIDE_timer = 0;
uint8_t SendInfoStatus = 0;

uint8_t Docked = 0;
uint8_t MasterSwitch = 1;			// This is the "masterswitch", by default this is turned on.

char msg[68];						// Buffer to store all our messages
uint8_t PIBuffer[PI_BFR_SIZE];
uint8_t ConsoleBuffer[CONSOLE_BFR_SIZE];
uint8_t UART1_ready = 0;
uint8_t UART2_ready = 0;
uint8_t Security = 0;

uint8_t WorkingHourStart = 10;		// RazorBoard will not start mowing until we reached 10.00
uint8_t WorkingHourEnd = 20;		// RazorBoard will mow until we reached 20.00, after that it will go home
uint8_t Battery_Ready = 0;			// Battery fully charged?
uint16_t HoldChargeDetection = 200;	// When we detect connection with the charger, wait in milliseconds before we brake/stop.

uint8_t M1_idx = 0;
uint8_t M2_idx = 0;
uint8_t C1_idx = 0;
float M1_force[20];					// Array of power consumption of Motor M1, used for detecting if we bump into something
float M2_force[20];					// Array of power consumption of Motor M2, used for detecting if we bump into something
float C1_force[20];
float M1_F = 0;
float M2_F = 0;
float C1_F = 0;
uint8_t Force_Active = 0;			// When the mower has gained enough movement, Force_Active will turn on.

float hold_heading = 0;				// In degrees, something to hold on to and try to steer in a straight line
float heading = 0.0;				// Relative heading
float yaw_error = 0.0;
float roll = 0.00;
float roll_error = 0.0;
float pitch = 0.00;
float pitch_error = 0.0;
float pitch_limit[20] = {0};
float roll_limit[20] = {0};
uint8_t pitch_limit_idx = 0;
uint8_t roll_limit_idx = 0;
uint8_t Overturn_Limit = 35;

float32_t BWF1[ADC_SAMPLE_LEN / 2];
float32_t BWF2[ADC_SAMPLE_LEN / 2];
arm_fir_instance_f32 S;
uint8_t Signature_Record = FALSE;

uint32_t MotorSpeedUpdateFreq_timer = 0;	// Timer for MotorSpeed Update
uint8_t MotorSpeedUpdateFreq = 100;			// Freq to update motor speed, in milliseconds

uint8_t ChargerConnect = 0;					// Are we connected to the charger?

uint8_t DEBUG_RAZORBOARD = 0;				// Used by "debug on/debug off"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_IWDG_Init(void);
static void MX_RNG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

static void MotorStop(void);
static void MotorBrake(void);
static void MotorHardBrake(void);
static void Undock_MotorBackward(uint16_t minSpeed, uint16_t maxSpeed);
static void MotorForward(uint16_t minSpeed, uint16_t maxSpeed);
static void MotorBackward(uint16_t minSpeed, uint16_t maxSpeed, uint16_t time_ms);
static void MotorLeft(uint16_t minSpeed, uint16_t maxSpeed, uint16_t time_ms);
static void MotorRight(uint16_t minSpeed, uint16_t maxSpeed, uint16_t time_ms);
static void Serial_Console(char *msg);
static void Serial_RPi(char *msg);
static void CheckState(void);
static uint8_t CheckSecurity(void);
static void CheckBWF(void);
static void cutterON(void);
static void cutterOFF(void);
static void cutterHardBreak(void);
static void parseCommand_RPI(void);
static void parseCommand_Console(void);
static void perimeterTracker(void);
static void ChargerConnected(void);
void HAL_UART_RxCpltCallback_UART1(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback_UART2(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
static void delay_us(uint16_t us);
static void ADC_Send(uint8_t channel);
static int ADC_Receive();
static void CollectADC(void);
static void SendInfo(void);
static void CheckVoltage(void);
static void CheckMotorCurrent(int RAW);
static void ProcessIMUData();
static void UpdateMotorSpeed();
static void Init6050(void);
static void MPU6050_Read_Accel(void);
static void MPU6050_Read_Gyro(void);
static void unDock(void);
static void reInitIMU(void);
static uint32_t rnd(uint32_t maxValue);
static void InitFIR(void);
static void FIR_LEFT(void);
static void FIR_RIGHT(void);
static void WatchdogRefresh(void);
static void WatchdogInit(void);
static void (*SysMemBootJump) (void);
static void BootLoaderInit(unsigned long BootLoaderStatus);
static void setTime(uint8_t hour, uint8_t minute, uint8_t second);
static void setDate(uint8_t year, uint8_t month, uint8_t day, uint8_t weekday);
static void TimeToGoHome(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TimeToGoHome(void) {

	// Get Time and check if we should go home

	RTC_TimeTypeDef currTime = {0};
	RTC_DateTypeDef currDate = {0};

	HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);

	if (currTime.Hours >= WorkingHourEnd) {
		perimeterTracking = 1;
	}

}

void setTime(uint8_t hour, uint8_t minute, uint8_t second) {

	RTC_TimeTypeDef sTime = {0};

	sTime.Hours = RTC_ByteToBcd2(hour);
	sTime.Minutes = RTC_ByteToBcd2(minute);
	sTime.Seconds = RTC_ByteToBcd2(second);
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_SET;
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

}
void setDate(uint8_t year, uint8_t month, uint8_t day, uint8_t weekday) {

	RTC_DateTypeDef sDate = {0};

	sDate.WeekDay = RTC_ByteToBcd2(weekday);
	sDate.Month = RTC_ByteToBcd2(month);
	sDate.Date = RTC_ByteToBcd2(day);
	sDate.Year = RTC_ByteToBcd2(year);
	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

}

void BootLoaderInit(unsigned long BootLoaderStatus)

// Inside the debug menu you can type "upgrade", this will force the STM into the bootloader, so no need to change the jumper.
{
  SysMemBootJump = (void (*)(void)) (*((unsigned long *) 0x1fff0004));
  if (BootLoaderStatus == 1)
  {

    HAL_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    __set_PRIMASK(1);
    __set_MSP(0x20001000);
    SysMemBootJump();
    while(1);
  }
}

void WatchdogInit(void) {

    /*
        Watchdog freq: 32.768 kHz
        Prescaler: 16
        Reload: 4095

        Our Watchdog:

        16 * 4095 = 64
        64 / 32 = 2 seconds
        if the watchdog is not fed within 2 seconds, the Razorboard will simply reboot.
    */

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
    hiwdg.Init.Reload = 4095;
    HAL_IWDG_Init(&hiwdg);

}

void WatchdogRefresh(void) {

	// refresh the watchdog

    HAL_IWDG_Refresh(&hiwdg);
}

void InitFIR(void) {

	// Initiate the FIR functions

	uint32_t blocksize = BLOCK_SIZE;
	arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blocksize);
}

void FIR_LEFT(void) {
	  uint32_t i;
	  uint32_t blockSize = BLOCK_SIZE;
	  uint32_t numBlocks = LENGTH_SAMPLES/BLOCK_SIZE;
	  float32_t  *inputF32, *outputF32;
	  inputF32 = &BWF1[0];
	  outputF32 = &Output[0];
	  for(i=0; i < numBlocks; i++)
	  {
	    arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
	  }
	  for (int x = 0; x < 256; x++) {
		  BWF1[x] = (float)outputF32[x];
	  }
}
void FIR_RIGHT(void) {
	  uint32_t i;
	  uint32_t blockSize = BLOCK_SIZE;
	  uint32_t numBlocks = LENGTH_SAMPLES/BLOCK_SIZE;
	  float32_t  *inputF32, *outputF32;
	  inputF32 = &BWF2[0];
	  outputF32 = &Output[0];
	  for(i=0; i < numBlocks; i++)
	  {
	    arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
	  }
	  for (int x = 0; x < 256; x++) {
		  BWF2[x] = (float)outputF32[x];
	  }
}

uint32_t rnd(uint32_t maxValue) {

	// Our random number generator

	uint32_t rndnum;
	rndnum = HAL_RNG_GetRandomNumber(&hrng) % maxValue;
	return rndnum;
}

void reInitIMU(void) {

	// If the I2C bus hangs, this will clear the deadlock and re-init the MPU

	Serial_Console("reInit IMU\r\n");

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_I2C_DeInit(&hi2c2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_Delay(1);
	MX_I2C2_Init();
	Init6050();
}



void MPU6050_Read_Gyro(void) {

	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	if (HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 50) != HAL_OK) {
		reInitIMU();	// if the MPU-6050 does not respond within 50ms, re-init
		return;
	}

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;

	if (Initial_Start == 0) {
		if (fabs(Gz) < 1.0) {
			yaw_error = fabs(Gz);		// Auto calibrate the Gyro error at startup
		}
	}
	Gz += yaw_error;					//Cancel out error

	if (Gz < -1.00 || Gz > 1.00) {		//if not moving, do not change
	yaw = yaw - (Gz * 0.02);
	}

}

void MPU6050_Read_Accel(void) {

	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	if (HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 50) != HAL_OK) {
		reInitIMU();		// if the MPU-6050 does not respond within 50ms, re-init
		return;
	}

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;

	float r, p;
	r = atan2(Ay , Az) * 57.3;
	p = atan2((- Ax) , sqrtf(Ay * Ay + Az * Az)) * 57.3;

	raw_roll = r + roll_error;
	raw_pitch = p + pitch_error;

}

void Init6050() {

	uint8_t check;
	uint8_t Data;

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 50);
	HAL_Delay(20);

	Data = 0;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 50);
	HAL_Delay(20);

	Data = 0x07;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 50);
	HAL_Delay(20);

	Data = 0;
	HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 50);
	HAL_Delay(20);

}

void ProcessIMUData() {

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

	roll = r / 20;
	pitch = p / 20;

	if (yaw > 359.9) yaw = 0;
	if (yaw < 0) yaw = 359.9;

	heading = yaw;

}

void CheckMotorCurrent(int RAW) {

			// Check if any motor is experiencing a spike in power, then we probably hit something.

			float M1, M2;
	        if (M1_idx == 10 || M2_idx == 10 || C1_idx == 10) Force_Active = 1;

			if (Channel == M1_addr) {
				M1_Value = RAW;
	            M1 = fabs(((M1_Value * 0.1875) - 2500) / 100);
	            if (Initial_Start == 0) M1_error = M1;
	            M1_amp = fabs(M1 - M1_error);
	            M1_force[M1_idx] = M1_amp;
	            M1_idx++;
	            if (M1_idx == 20) M1_idx = 0;
	            float ForceM1 = 0;
	            for (int x = 0; x < 20; x++) {
	            	ForceM1 += M1_force[x];
	            }
	            M1_F = ForceM1 / 20;
	            if (M1_F < 0.1) M1_F = 0.1;
	            if ((M1_amp >= 0.3 || M1_amp >= M1_F * Motor_Limit) && State == (FORWARD || RIGHT) && Force_Active == 1) {

	            	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8 ) == GPIO_PIN_SET) {
	            		return;
	            	}
	            	MotorBrake();
	            	HAL_Delay(500);
	            	MotorBackward(MotorMinSpeed, MotorMaxSpeed, 400);
	            	HAL_Delay(500);
	            	MotorRight(MotorMinSpeed, MotorMaxSpeed, 300);
	            	Force_Active = 0;

	            }
			}
			else if (Channel == M2_addr) {
				M2_Value = RAW;
	            M2 = fabs(((M2_Value * 0.1875) - 2500) / 100);
	            if (Initial_Start == 0) M2_error = M2;
	            M2_amp = fabs(M2 - M2_error);
	            M2_force[M2_idx] = M2_amp;
	            M2_idx++;
	            if (M2_idx == 20) M2_idx = 0;
	            float ForceM2 = 0;
	            for (int x = 0; x < 20; x++) {
	            	ForceM2 += M2_force[x];
	            }
	            M2_F = ForceM2 / 20;
	            if (M2_F < 0.1) M2_F = 0.1;
	            if ((M2_amp >= 0.3 || M2_amp >= M2_F * Motor_Limit) && State == (FORWARD || LEFT) && Force_Active == 1) {

	            	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8 ) == GPIO_PIN_SET) {
	            		return;
	            	}
	            	MotorBrake();
	            	HAL_Delay(500);
	            	MotorBackward(MotorMinSpeed, MotorMaxSpeed, 400);
	            	HAL_Delay(500);
	            	MotorLeft(MotorMinSpeed, MotorMaxSpeed, 300);
	            	Force_Active = 0;

	            }
			}
}

void CheckVoltage() {

	//Simple Voltage check

	if ( Voltage < Voltage_Limit_LOW) {
		if (perimeterTracking == 1) return;
		sprintf(msg, "Low Voltage - Searching for perimeter wire...\r\n");
		Serial_RPi(msg);
		Serial_Console(msg);
		perimeterTracking = 1;
		cutterOFF();
	}

}

void SendInfo() {

		SendInfoStatus = 0;

		if (DEBUG_RAZORBOARD == 0) {
			bwf1_inside = 0;
			bwf1_outside = 0;
			bwf2_inside = 0;
			bwf2_outside = 0;
			return;
		}

		// Send info to the Console & Raspberry PI - Update frequency is ~1 Hz
    	RTC_TimeTypeDef currTime = {0};
    	RTC_DateTypeDef currDate = {0};

		HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);

		sprintf(msg, "M1: %.2f\r\n", M1_amp);
		Serial_Console(msg);
		sprintf(msg, "M2: %.2f\r\n", M2_amp);
		Serial_Console(msg);
		sprintf(msg, "C1: %.2f\r\n", C1_amp);
		Serial_Console(msg);
		sprintf(msg, "V1: %.2f\r\n", (Voltage / 1000));
		Serial_Console(msg);
		sprintf(msg,"Charger Connected: %d\r\n", ChargerConnect);
		Serial_Console(msg);
		sprintf(msg,"IN-> BWF1: %d BWF2: %d\r\nOUT-> BWF1: %d BWF2: %d\r\n", bwf1_inside, bwf2_inside, bwf1_outside, bwf2_outside);
		Serial_Console(msg);
		sprintf(msg, "Battery Fully Charged: %d\r\n", Battery_Ready);
		Serial_Console(msg);
		sprintf(msg, "Roll: %.2f Pitch: %2.f Yaw: %2.f\r\n", roll, pitch, yaw);
		Serial_Console(msg);
		sprintf(msg, "Time: %d:%d:%d\r\n", currTime.Hours, currTime.Minutes, currTime.Seconds);
		Serial_Console(msg);
		sprintf(msg, "Date: 20%d-%d-%d\r\n", currDate.Year, currDate.Month, currDate.Date);
		Serial_Console(msg);

		char Data[128];

		sprintf(Data, "Battery Voltage,%.2f\r\n"
				      "M1 Current,%.2f\r\n"
				      "M2 Current,%.2f\r\n"
				      "C1 Current,%.2f\r\n"
				      "Security,%d\r\n"
				      "Mower State,%d\r\n", Voltage, M1_amp,M2_amp,C1_amp, Security, State);

		bwf1_inside = 0;
		bwf1_outside = 0;
		bwf2_inside = 0;
		bwf2_outside = 0;

		HAL_UART_Transmit(&huart2, (uint8_t *)&Data, strlen(Data), 100);

}

void CollectADC() {

	// Collect IC2 data from the external ADC, in a non-blocking way.

	if (Channel == M1_addr && Channel_Status == 0) {
		ADC_Send(Channel);
		Channel_Status = 1;
		ADC_timer = HAL_GetTick();
	}
	else if (Channel == M2_addr && Channel_Status == 0) {
		ADC_Send(Channel);
		Channel_Status = 1;
		ADC_timer = HAL_GetTick();
	}
	else if (Channel == C1_addr && Channel_Status == 0) {
		ADC_Send(Channel);
		Channel_Status = 1;
		ADC_timer = HAL_GetTick();
	}
	else if (Channel == V1_addr && Channel_Status == 0) {
		ADC_Send(Channel);
		Channel_Status = 1;
		ADC_timer = HAL_GetTick();
	}

	if ((HAL_GetTick() - ADC_timer) >= 20 && Channel_Status == 1) {
		int RAW = 0;
		RAW = ADC_Receive();

		CheckMotorCurrent(RAW);

	    if (Channel == C1_addr) {
			C1_Value = RAW;
			float C1;
            C1 = fabs(((C1_Value * 0.1875) - 2500) / 100);
            if (Initial_Start == 0) C1_error = C1;
            C1_amp = fabs(C1 - C1_error);
            if (C1_amp >= Cutter_Limit) {
            	MasterSwitch =  0;
            	return;
            }

		}
		else if (Channel == V1_addr) {
			V1_Value = RAW;
            Voltage = (V1_Value * 0.1875) * 5.0;
		}

		if (Channel == M1_addr) Channel = M2_addr;
		else if (Channel == M2_addr) Channel = C1_addr;
		else if (Channel == C1_addr) Channel = V1_addr;
		else if (Channel == V1_addr) Channel = M1_addr;
		Channel_Status = 0;

	}
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim5,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim5) < us);  // wait for the counter to reach the us input in the parameter
}

void unDock(void) {

	// Simple undock sequence,  check if Battery is ready (fully charged) and if we are within working hours.

	RTC_TimeTypeDef currTime = {0};
	RTC_DateTypeDef currDate = {0};

	HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);

	if (currTime.Hours >= WorkingHourStart && currTime.Hours < WorkingHourEnd && Battery_Ready == 1 && Docked == 1) {

		Serial_Console("Switching to Main Battery\r\n");
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_Delay(5000);

		Undock_MotorBackward(MotorMinSpeed, MotorMaxSpeed);

		MotorLeft(MotorMinSpeed, MotorMaxSpeed, 500);			// This needs to be changed if your docking is on the right side

		Docked = 0;
		Initial_Start = 0;
		Start_Threshold = 0;
		Battery_Ready = 0;
		lastError = 0;

	}

}

void ChargerConnected(void) {

	// Is the charger connected?

	if (ChargerConnect == 1 || Docked == 1) {
		if (Voltage >= Voltage_Fully_Charged && Battery_Ready == 0) {
			Battery_Ready = 1;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
			Serial_Console("Charger disconnected.\r\n");
			ChargerConnect = 0;
		}
		return;
	}

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8 ) == GPIO_PIN_SET) {			// Read Volt sense pin
		HAL_Delay(HoldChargeDetection);									// Wait for a while so a proper connection is made
		Force_Active = 0;
		MotorBrake();
		cutterHardBreak();
		Serial_Console("Charger Connected\r\n");
		HAL_Delay(10000);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);				// Main Power switch
		Serial_Console("Changing Main Power\r\n");
		ChargerConnect = 1;
		Docked = 1;
		HAL_Delay(5000);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);			// Charger Switch
		Serial_Console("Charging activated\r\n");
		perimeterTracking = 0;
		perimeterTrackingActive = 0;
		return;
		}
}

void perimeterTracker() {

	CheckSecurity();

	elapsedTime = HAL_GetTick() - previousTime;

    if (BWF2_Status == OUTSIDE) {
    	Tick1 -= elapsedTime;
    	Tick2 = 0;
    }

    if (BWF2_Status == INSIDE) {
    	Tick2 -= elapsedTime;
    	Tick1 = 0;
    }

    error = 2800 - (Tick1 + Tick2);                // determine error
    cumError += error * elapsedTime;               // compute integral
    rateError = (error - lastError)/elapsedTime;   // compute derivative

    double out = kp*error + ki*cumError + kd*rateError;                //PID output

    lastError = error;                             //remember current error
    previousTime = HAL_GetTick();                  //remember current time

    int speedA = (2800 + round(out));
    int speedB = (2800 - round(out));

    if (speedA > 3200) speedA = 3200;				// limit upper and lower speed
    if (speedB > 3200) speedB = 3200;

    if (speedA < 1000) speedA = 1000;
    if (speedB < 1000) speedB = 1000;

    if (BWF2_Status == OUTSIDE) {

		  if (BWF1_Status == OUTSIDE) {
			  TIM4->CCR1 = 2300;			// if both boundary sensors are OUTSIDE, reverse M1 motor, this logic needs to be changed if docking is to the right
			  TIM4->CCR2 = 0;
		  }
		  else if (BWF1_Status == INSIDE) {
			  TIM4->CCR1 = 0;
			  TIM4->CCR2 = speedB;
		  }

		  TIM4->CCR3 = speedA;
		  TIM4->CCR4 = 0;

    }

    if (BWF2_Status == INSIDE) {
		  TIM4->CCR1 = 0;
		  TIM4->CCR2 = speedA;

		  TIM4->CCR3 = round(speedB * 0.95);
		  TIM4->CCR4 = 0;
    }

}
void parseCommand_Console(void) {

	// Parse commands from the Console

	char Command[64] = {"\0"};

	for (uint8_t x = 0; x < sizeof(ConsoleBuffer); x++) {
		if (ConsoleBuffer[x] == 13) {
			if (ConsoleBuffer[0] == 13) memcpy(Command, "DISABLE", 7);
			else {
				memcpy(Command,ConsoleBuffer,x);
			}
			sprintf(msg,"%s\r\n", Command);
			Serial_Console(msg);
			for (uint8_t i = 0; i < CONSOLE_BFR_SIZE; i++) {
				Command[i] = toupper(Command[i]);
			}

			if (strcmp(Command, "HELLO") == 0) {
				Serial_Console("Hello fellow user! Welcome to RazorBoard!\r\n");
				}
			if (strcmp(Command, "VERSION") == 0) {
				sprintf(msg, "%s\r\n", VERSION);
				Serial_Console(msg);
				}
			if (strcmp(Command, "DEBUG ON") == 0) {
				DEBUG_RAZORBOARD = 1;
				Serial_Console("DEBUG is now ON\r\n");
				}
			if (strcmp(Command, "DEBUG OFF") == 0) {
				DEBUG_RAZORBOARD = 0;
				Serial_Console("DEBUG is now OFF\r\n");
				}
			if (strcmp(Command, "VOLTAGE") == 0) {
				float volt = Voltage / 1000;
				sprintf(msg, "%.2f\r\n", volt);
				Serial_Console(msg);
				}
			if (strcmp(Command, "UPGRADE") == 0) {
				Serial_Console("Entering Bootloader...\r\n");
				HAL_Delay(500);
				BootLoaderInit(1);
			}
			if (strcmp(Command, "SHOW SIG") == 0) {
				Serial_Console("Signature will be printed in 8 seconds, please start your plotter before.\r\n");
				HAL_Delay(8000);
				for (int x = 0; x < SIGNATURE_LEN; x++) {
					sprintf(msg,"%f\r\n", validSignature[x]);
					Serial_Console(msg);
				}
			}
			if (strcmp(Command, "EXPORT SIG") == 0) {
				Serial_Console("Signature exported as an array\r\n");
				Serial_Console("{ ");
				for (int x = 0; x < SIGNATURE_LEN; x++) {
					sprintf(msg,"%f,", validSignature[x]);
					Serial_Console(msg);
				}
				Serial_Console(" };\r\n");
			}
			if (strcmp(Command, "RECORD SIG") == 0) {
				Signature_Record = TRUE;
				Serial_Console("Done!\r\n");
			}
			if (strcmp(Command, "TEST LEFT MOTOR") == 0) {
				  TIM4->CCR1 = 0;
				  TIM4->CCR2 = 2000;
				  HAL_Delay(3000);
				  MotorStop();
				  Serial_Console("Done.\r\n");
			}
			if (strcmp(Command, "TEST RIGHT MOTOR") == 0) {
				  TIM4->CCR3 = 2000;
				  TIM4->CCR4 = 0;
				  HAL_Delay(3000);
				  MotorStop();
				  Serial_Console("Done.\r\n");
			}
			if (strcmp(Command, "REBOOT") == 0) {
				Serial_Console("Rebooting...\r\n");
				HAL_Delay(500);
				NVIC_SystemReset();

			}
			if (strcmp(Command, "SHOW CURRENT") == 0) {
				sprintf(msg, "M1: %.2f\r\nM2: %.2f\r\nC1: %.2f\r\n", M1_amp, M2_amp, C1_amp);
				Serial_Console(msg);
			}
			if (strcmp(Command, "STOP MOTORS") == 0) {
				MotorStop();
			}
			if (strcmp(Command, "RUN MOTORS FORWARD") == 0) {
				TIM4->CCR1 = 0;
				TIM4->CCR2 = 2000;
				TIM4->CCR3 = 2000;
				TIM4->CCR4 = 0;
			}
			if (strcmp(Command, "RUN MOTORS REVERSE") == 0) {
				TIM4->CCR1 = 2000;
				TIM4->CCR2 = 0;
				TIM4->CCR3 = 0;
				TIM4->CCR4 = 2000;
			}
			if (strncmp(Command, "SET DATE", 8) == 0) {
				int year = 0, month = 0, day = 0, weekday = 0;
				char cmd1[3], cmd2[4];
				sscanf(Command, "%s %s %d %d %d %d", cmd1, cmd2, &year, &month, &day, &weekday);
				setDate(year, month, day, weekday);

			}
			if (strncmp(Command, "SET TIME", 8) == 0) {
				int hour = 0, minute = 0, second = 0;
				char cmd1[3], cmd2[4];
				sscanf(Command, "%s %s %d %d %d", cmd1, cmd2, &hour, &minute, &second);
				setTime(hour, minute, second);
			}
			if (strncmp(Command, "SET KP", 6) == 0) {
				float pid_kp;
				char cmd1[3], cmd2[2];
				sscanf(Command, "%s %s %f", cmd1, cmd2, &pid_kp);
				kp = pid_kp;
			}
			if (strncmp(Command, "SET KI", 6) == 0) {
				float pid_ki;
				char cmd1[3], cmd2[2];
				sscanf(Command, "%s %s %f", cmd1, cmd2, &pid_ki);
				ki = pid_ki;
			}
			if (strncmp(Command, "SET KD", 6) == 0) {
				float pid_kd;
				char cmd1[3], cmd2[2];
				sscanf(Command, "%s %s %f", cmd1, cmd2, &pid_kd);
				kd = pid_kd;
			}

			if (strcmp(Command, "DISABLE") == 0) {
				MasterSwitch = 0;
				Serial_Console("RazorBoard DISABLED.\r\n");
				Serial_Console("Please type <help> to see available commands\r\n");
			}

			if (strcmp(Command, "ENABLE") == 0) {
				MasterSwitch = 1;
				Serial_Console("RazorBoard ENABLED. STEP AWAY FROM THE VEHICLE!\r\n");
				Initial_Start = 0;
				Initial_Start = 0;
			}
			if (strcmp(Command, "TRACK PERIMETER") == 0) {
				perimeterTracking = 1;
				cutterOFF();
				Serial_Console("Perimeter tracking ENABLED\r\n");
			}
			if (strcmp(Command, "SHOW PID TRACKING") == 0) {
				sprintf(msg, "KP: %f KI: %f KD: %f\r\n", kp, ki, kd);
				Serial_Console(msg);
			}
			if (strncmp(Command, "SET WORKING START", 17) == 0) {
				int start;
				char cmd1[3], cmd2[7], cmd3[5];
				sscanf(Command, "%s %s %s %d", cmd1, cmd2, cmd3, &start);
				WorkingHourStart = start;
			}
			if (strncmp(Command, "SET WORKING END", 15) == 0) {
				int end;
				char cmd1[3], cmd2[7], cmd3[3];
				sscanf(Command, "%s %s %s %d", cmd1, cmd2, cmd3, &end);
				WorkingHourEnd = end;
			}
			if (strcmp(Command, "SHOW WORKINGHOURS") == 0) {
				sprintf(msg, "START: %d END: %d\r\n", WorkingHourStart, WorkingHourEnd);
				Serial_Console(msg);
			}
			if (strcmp(Command, "HELP") == 0) {
				sprintf(msg, "Available commands:\r\n\r\n");
				Serial_Console(msg);
				sprintf(msg, "HELLO             	- Welcome message\r\n");
				Serial_Console(msg);
				sprintf(msg, "REBOOT                  - Reboot Razorboard\r\n");
				Serial_Console(msg);
				sprintf(msg, "DISABLE                 - Disable Razorboard\r\n");
				Serial_Console(msg);
				sprintf(msg, "ENABLE                  - Enable Razorboard\r\n");
				Serial_Console(msg);
				sprintf(msg, "VERSION           	- Show version of board\r\n");
				Serial_Console(msg);
				sprintf(msg, "DEBUG ON          	- Enable debug messages\r\n");
				Serial_Console(msg);
				sprintf(msg, "DEBUG OFF         	- Disable debug messages\r\n");
				Serial_Console(msg);
				sprintf(msg, "VOLTAGE           	- Show current voltage\r\n");
				Serial_Console(msg);
				sprintf(msg, "UPGRADE           	- Enter bootloader\r\n");
				Serial_Console(msg);
				sprintf(msg, "SHOW SIG          	- Show reference BWF signature\r\n");
				Serial_Console(msg);
				sprintf(msg, "EXPORT SIG          	- Export reference BWF signature as an array\r\n");
				Serial_Console(msg);
				sprintf(msg, "RECORD SIG              - Record a new signature\r\n");
				Serial_Console(msg);
				sprintf(msg, "TEST LEFT MOTOR   	- Test left motor (M1)\r\n");
				Serial_Console(msg);
				sprintf(msg, "TEST RIGHT MOTOR  	- Test right motor (M2)\r\n");
				Serial_Console(msg);
				sprintf(msg, "SHOW CURRENT      	- Show current sensors M1, M2, C1\r\n");
				Serial_Console(msg);
				sprintf(msg, "STOP MOTORS       	- Stop motors\r\n");
				Serial_Console(msg);
				sprintf(msg, "RUN MOTORS FORWARD	- Run motors forward\r\n");
				Serial_Console(msg);
				sprintf(msg, "RUN MOTORS REVERSE	- Run motors backward\r\n");
				Serial_Console(msg);
				sprintf(msg, "SET TIME		- Set current time for RTC\r\n");
				Serial_Console(msg);
				sprintf(msg, "SET DATE		- Set current date for RTC\r\n");
				Serial_Console(msg);
				sprintf(msg, "			Date must be set in a special order:\r\n");
				Serial_Console(msg);
				sprintf(msg, "			Year Month Day Weekday -> 21 3 31 2 (2 = Wednesday)\r\n");
				Serial_Console(msg);
				sprintf(msg, "TRACK PERIMETER 	- Track perimeter next time it crosses\r\n");
				Serial_Console(msg);
				sprintf(msg, "SET KP			- PID Controller KP for Perimeter Tracking\r\n");
				Serial_Console(msg);
				sprintf(msg, "SET KI			- PID Controller KI for Perimeter Tracking\r\n");
				Serial_Console(msg);
				sprintf(msg, "SET KD			- PID Controller KD for Perimeter Tracking\r\n");
				Serial_Console(msg);
				sprintf(msg, "SHOW PID TRACKING	- Export current settings for tracking PID\r\n");
				Serial_Console(msg);
				sprintf(msg, "SET WORKING START	- Set Working Hour START\r\n");
				Serial_Console(msg);
				sprintf(msg, "SET WORKING END   - Set Working Hour END\r\n");
				Serial_Console(msg);
				sprintf(msg, "SHOW WORKINGHOURS	- Show current set working hours\r\n");
				Serial_Console(msg);
			}

			memset(ConsoleBuffer, 0, sizeof(CONSOLE_BFR_SIZE));

			UART1_ready = 0;
			break;
			}
	}
	UART1_ready = 0;
}
void parseCommand_RPI() {

	// Parse commands from the Raspberry PI

	char Command[64] = {"\0"};

	for (uint8_t x = 0; x < sizeof(PIBuffer); x++) {
		if (PIBuffer[x] == 13) {
			memcpy(Command,PIBuffer,x);
			sprintf(msg,"%s\r\n", Command);
			if (strcmp(Command, "RUN") == 0) {
				MasterSwitch = 1;
				Serial_RPi("Status: RUN\r\n");
				Serial_Console("Status: RUN\r\n");
			}
			if (strcmp(Command, "AVOID_LEFT") == 0 && State == FORWARD) {
				AVOID_LEFT = 1;
			}
			if (strcmp(Command, "AVOID_RIGHT") == 0 && State == FORWARD) {
				AVOID_RIGHT = 1;
			}
			if (strcmp(Command, "AVOID_FRONT") == 0 && State == FORWARD) {
				AVOID_FRONT = 1;
			}
			if (strcmp(Command, "GOHOME") == 0) {
				perimeterTracking = 1;
			}
			if (strcmp(Command, "STOP") == 0) {
				MasterSwitch = 0;

				Serial_RPi("Status: STOP\r\n");
				Serial_Console("Status: STOP\r\n");
			}
			Serial_Console(msg);
			UART2_ready = 0;

			break;
		}
	}
}

void Serial_Console(char *msg) {

	// Write to USB/Serial

	HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
}
void Serial_RPi(char *msg) {

	// Write to Raspberry PI

	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
}
uint8_t CheckSecurity(void) {

	// Check security, what is our status with the boundary signals

    CheckBWF();


    if (HAL_GetTick() - Boundary_Timer >= (Boundary_Timeout * 1000)) {
    	BWF1_Status = NOSIGNAL;
    	BWF2_Status = NOSIGNAL;
    	State = FAIL;
    	Security = NOSIGNAL;
    	return SECURITY_NOSIGNAL;
    }

    if (BWF1_Status == INSIDE && BWF2_Status == INSIDE) {
    	Security = INSIDE;
    	OUTSIDE_timer = HAL_GetTick();		// We are inside, reset OUTSIDE_timer
		return SECURITY_OK;
	}
	else if (BWF1_Status == OUTSIDE || BWF2_Status == OUTSIDE) {
		Security = OUTSIDE;
		return SECURITY_FAIL;
	}

	return SECURITY_FAIL;

}

void cutterHardBreak() {

	// Cutter disc hard brake

	TIM3->CCR1 = MotorMaxSpeed;		// Motor will hard brake when both "pins" go HIGH
	TIM3->CCR2 = MotorMaxSpeed;
	HAL_Delay(3000);
	cutterOFF();

}

void cutterON(void) {

	cutterStatus = 1;

	Serial_Console("Cutter Motor ON\r\n");

	if (rnd(1000) < 500 ) {			// Randomly select CW or CCW

		for (uint16_t cutterSpeed = 1000; cutterSpeed < CUTTER_MAX_SPEED; cutterSpeed++) {

			Boundary_Timer = HAL_GetTick();
			TIM3->CCR1 = cutterSpeed;
			TIM3->CCR2 = 0;
			HAL_Delay(2);

		}
	}
	else {
		for (uint16_t cutterSpeed = 1000; cutterSpeed < CUTTER_MAX_SPEED; cutterSpeed++) {

			Boundary_Timer = HAL_GetTick();
			TIM3->CCR1 = 0;
			TIM3->CCR2 = cutterSpeed;
			HAL_Delay(2);

		}
	}

}

void cutterOFF(void) {

	cutterStatus = 0;

	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;

}

void CheckBWF() {

	/*
	 * From the 512 samples, break out 256 for each sensor
	 * Run FIR filter
	 * Run Cross-Correlation to find a signal match.
	 * 1.0 = 100% match for INSIDE
	 * 0.85 = 85% match for INSIDE
	 * -0.75 = 75% match for OUTSIDE
	 * -1.0 = 100% match for OUTSIDE
	 */

	float BWF1_Mixed_Signal = 0;
	float BWF1_Received_Signal = 0;
	float BWF2_Mixed_Signal = 0;
	float BWF2_Received_Signal = 0;
	uint16_t myID = 0;
	uint8_t BWF1_reply, BWF2_reply = 0;
	float Match_Signal = 0;
	float Result_Signal = 0;
	float BWF1_Verdict_Signal = 0.0;
	float BWF2_Verdict_Signal = 0.0;
	int count = 0;

	for (int x = 0; x < ADC_SAMPLE_LEN; x++) {
		if (x%2) {
			BWF2[count] = ADC_BUFFER[x] - 1267;		// Normalize the ADC signal
			count++;
		}
		else {
			BWF1[count] = ADC_BUFFER[x] - 1267;		// Normalize the ADC signal
		}
	}

	FIR_LEFT();		// Run FIR on left BWF	(BWF1)
	FIR_RIGHT();	// Run FIR on right BWF	(BWF2)

	if (Signature_Record == TRUE) {
		for (uint8_t x = 0; x < SIGNATURE_LEN; x++) {
			validSignature[x] = BWF1[x];
		}
		Signature_Record = FALSE;
	}

	for (uint16_t idx = 0; idx < 96; idx++) {
        if (BWF1_reply == 1 && BWF2_reply == 1) {
        	break;
        }
		myID = 0;
		BWF1_Mixed_Signal = 0;
		BWF1_Received_Signal = 0;
		BWF2_Mixed_Signal = 0;
		BWF2_Received_Signal = 0;
		Match_Signal = 0;


		for (int x = idx; x < (idx+SIGNATURE_LEN - 1); x++) {

			BWF1_Mixed_Signal += (BWF1[x] * validSignature[myID]);
			BWF1_Received_Signal += BWF1[x] * BWF1[x];

			BWF2_Mixed_Signal += (BWF2[x] * validSignature[myID]);
			BWF2_Received_Signal += BWF2[x] * BWF2[x];

			Match_Signal += validSignature[myID] * validSignature[myID];
			myID++;
		}

	Result_Signal = sqrtf(BWF1_Received_Signal * Match_Signal);
	BWF1_Verdict_Signal = (BWF1_Mixed_Signal / Result_Signal);

	Result_Signal = sqrtf(BWF2_Received_Signal * Match_Signal);
	BWF2_Verdict_Signal = (BWF2_Mixed_Signal / Result_Signal);

    if (BWF1_Verdict_Signal >= Signal_Integrity_IN) {
    	BWF1_Status = INSIDE;
    	Boundary_Timer = HAL_GetTick();
    	BWF1_reply = 1;
    	if (Initial_Start == 0) {
    		Start_Threshold++;
    		if (Start_Threshold >= INITIAL_MAX_THRESHOLD) {
    			Initial_Start = 1;
    		}
    	}
    	bwf1_inside++;

    }

    else if (BWF1_Verdict_Signal <= Signal_Integrity_OUT) {
    	BWF1_Status = OUTSIDE;
    	Boundary_Timer = HAL_GetTick();
    	BWF1_reply = 1;
    	if (Initial_Start == 0) Start_Threshold = 0;
    	bwf1_outside++;

    }

    if (BWF2_Verdict_Signal >= Signal_Integrity_IN) {
    	BWF2_Status = INSIDE;
    	Boundary_Timer = HAL_GetTick();
    	BWF2_reply = 1;
    	if (Initial_Start == 0) {
    		Start_Threshold++;
    		if (Start_Threshold >= INITIAL_MAX_THRESHOLD) {
    			Initial_Start = 1;
    		}
    	}
    	bwf2_inside++;

    }
    else if (BWF2_Verdict_Signal <= Signal_Integrity_OUT) {
    	BWF2_Status = OUTSIDE;
    	Boundary_Timer = HAL_GetTick();
    	BWF2_reply = 1;
    	if (Initial_Start == 0) Start_Threshold = 0;
    	bwf2_outside++;

    }
	}
}

void ADC_Send(uint8_t channel) {

	// Send ADC data

unsigned char ADSwrite[6];

ADSwrite[0] = 0x01;
ADSwrite[1] = channel;
ADSwrite[2] = 0x83;
HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);
ADSwrite[0] = 0x00;
HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 1, 100);

}

int ADC_Receive() {

	// Receive ADC data

unsigned char ADSwrite[6];
int reading;

				HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS <<1, ADSwrite, 2, 100);

				reading = (ADSwrite[0] << 8 | ADSwrite[1] );

				if(reading < 0 || reading > 32768) {
					reading = 0;
				}

		return reading;

}

void UpdateMotorSpeed() {

	/* With the use of the MPU6050, we calculate the angle we are traveling in by using the gyro
	 * A simple motor speed controller, for each degree off, we simply decrease the speed with 120.
	 * TODO - a proper PID controller
	 */

	int16_t diff;
	int8_t dir = 0;

	// Calculate the difference in bearing, 0-360 accounted for. (Circular heading)
	diff = (((((int)heading - (int)hold_heading) % 360) + 540 ) % 360) - 180;
	diff *= 120.0;

	if (diff < 0) dir = -1;
	else if (diff > 0) dir = 1;
	else if (diff == 0) dir = 0;

		// Do not change speed more than 1000
	if (diff > 1000) diff = 1000;
	if (diff < -1000) diff = -1000;

		// Target is on the Left side
		if (dir > 0) {
			int CorrectedSpeed = MotorMaxSpeed - abs(diff);
			TIM4->CCR2 = CorrectedSpeed;
			TIM4->CCR3 = MotorMaxSpeed;
		}
		// Target is on the Right side
		else if (dir < 0){
			int CorrectedSpeed = MotorMaxSpeed - abs(diff);
			TIM4->CCR2 = MotorMaxSpeed;
			TIM4->CCR3 = CorrectedSpeed;

		}
		// Spot on! Full speed ahead Captain!
		else if (dir == 0) {
			TIM4->CCR2 = MotorMaxSpeed;
			TIM4->CCR3 = MotorMaxSpeed;
		}

}
void Undock_MotorBackward(uint16_t minSpeed, uint16_t maxSpeed) {

	State = BACKWARD;

for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {

	  currentSpeed += 1;
	  if (currentSpeed >= maxSpeed) {
		  break;
	  }

	  TIM4->CCR1 = currentSpeed;
	  TIM4->CCR2 = 0;

	  TIM4->CCR3 = 0;
	  TIM4->CCR4 = currentSpeed;

	  HAL_Delay(1);

 }
HAL_Delay(2000);
MotorStop();
}

void MotorForward(uint16_t minSpeed, uint16_t maxSpeed) {

	State = FORWARD;

	MPU6050_Read_Gyro();		// Get fresh data for Pitch/Roll
	ProcessIMUData();			// Compute Pitch/Roll

for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {

	  currentSpeed += 3;
	  if (currentSpeed >= maxSpeed) {
		  break;
	  }

	  uint16_t leftTilt = 0;
	  uint16_t rightTilt = 0;

	  if (roll > 0) {
		  leftTilt = fabs(roll * 50);
	  }
	  if (roll < 0) {
		  rightTilt = fabs(roll * 50);
	  }

	  TIM4->CCR1 = 0;
	  TIM4->CCR2 = currentSpeed - round(leftTilt);

	  TIM4->CCR3 = currentSpeed - round(rightTilt);
	  TIM4->CCR4 = 0;

	  HAL_Delay(1);

	  if (CheckSecurity() == SECURITY_FAIL) {
		  MotorStop();
		  break;
	  }

 }
}
void MotorBackward(uint16_t minSpeed, uint16_t maxSpeed, uint16_t time_ms) {

	State = BACKWARD;
	uint16_t timeCount = 0;

for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {

	  currentSpeed += 3;
	  if (currentSpeed >= maxSpeed) {
		  break;
	  }

	  TIM4->CCR1 = currentSpeed;
	  TIM4->CCR2 = 0;

	  TIM4->CCR3 = 0;
	  TIM4->CCR4 = currentSpeed;

	  HAL_Delay(1);
	  timeCount++;

	  CheckSecurity();

	  if (timeCount >= time_ms) {
		  break;
	  }
 }
MotorStop();
}
void MotorRight(uint16_t minSpeed, uint16_t maxSpeed, uint16_t time_ms) {

	State = RIGHT;
	uint16_t timeCount = 0;

for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {

	  currentSpeed += 3;
	  if (currentSpeed >= maxSpeed) {
		  break;
	  }
	  TIM4->CCR1 = 0;
	  TIM4->CCR2 = currentSpeed;

	  TIM4->CCR3 = 0;
	  TIM4->CCR4 = currentSpeed;

	  HAL_Delay(1);
	  timeCount++;

	  CheckSecurity();

	  if (timeCount >= time_ms) {
		  break;
	  }
 }
MotorStop();
}
void MotorLeft(uint16_t minSpeed, uint16_t maxSpeed, uint16_t time_ms) {

	State = LEFT;
	uint16_t timeCount = 0;

for (uint16_t currentSpeed = minSpeed; currentSpeed < maxSpeed; currentSpeed++) {

	  currentSpeed += 3;
	  if (currentSpeed >= maxSpeed) {
		  break;
	  }
	  TIM4->CCR1 = currentSpeed;
	  TIM4->CCR2 = 0;

	  TIM4->CCR3 = currentSpeed;
	  TIM4->CCR4 = 0;

	  HAL_Delay(1);
	  timeCount++;

	  CheckSecurity();

	  if (timeCount >= time_ms) {
		  break;
	  }
 }
MotorStop();
}
void MotorStop(void) {

	LastState = State;
	State = STOP;
	int speed = 0;

	speed = (TIM4->CCR1 + TIM4->CCR2 + TIM4->CCR3 + TIM4->CCR4) / 2;
	speed *= 0.90;

	if (speed == 0) return;

	for (int x = speed; x > 1000; x--) {

		x -= 4;

		if (x < 1000) break;

		if (TIM4->CCR1 != 0) TIM4->CCR1 = x;
		if (TIM4->CCR2 != 0) TIM4->CCR2 = x;
		if (TIM4->CCR3 != 0) TIM4->CCR3 = x;
		if (TIM4->CCR4 != 0) TIM4->CCR4 = x;

		HAL_Delay(1);
	}

	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;

}

void MotorBrake(void) {

	LastState = State;
	State = BRAKE;

	// Brake - free wheeling
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;

}
void MotorHardBrake(void) {

	LastState = State;

	State = HARDBRAKE;

	// Wheels will do a hard brake when both pins go HIGH.
	TIM4->CCR1 = 3360 -1;
	TIM4->CCR2 = 3360 -1;
	TIM4->CCR3 = 3360 -1;
	TIM4->CCR4 = 3360 -1;

	HAL_Delay(250);

	MotorBrake();	//Release motors

}
void CheckState(void) {

	/* This is our main function, this is where all the states are
	 * What state is the mower in? and what to do next.
	 */
	if (Initial_Start == 0) return;

	if (MasterSwitch == 0) {
		cutterOFF();
		MotorStop();
		return;
	}

	if (Security == NOSIGNAL) {
		MotorStop();
		cutterOFF();
		return;
	}
	if (HAL_GetTick() - OUTSIDE_timer >= (Outside_Threshold * 1000) && Docked == 0) {
		Serial_Console("OUTSIDE timer triggered\r\n");
		MotorStop();
		cutterOFF();
		return;
	}
	if (Docked == 1) {
		unDock();
		return;
		}

	if (State == FAIL) {
		MotorStop();
		cutterOFF();
		while (CheckSecurity() != SECURITY_OK) {
			Serial_Console("State Fail, waiting...\r\n");
		}
		State = STOP;
		return;
	}

	if (abs(pitch) >= Overturn_Limit || abs(roll) >= Overturn_Limit) {
		Serial_Console("Overturn\r\n");
		MotorHardBrake();
		cutterHardBreak();
		return;
	}

	else if (AVOID_LEFT == 1 || AVOID_RIGHT == 1 || AVOID_FRONT == 1) {
		MotorStop();
		Serial_Console("Obstacle\r\n");
		HAL_Delay(500);
		if (AVOID_LEFT == 1) {
			MotorRight(MotorMinSpeed, MotorMaxSpeed, 1000);
		}
		if (AVOID_RIGHT == 1) {
			MotorLeft(MotorMinSpeed, MotorMaxSpeed, 1000);
		}
		if (AVOID_FRONT == 1) {
			MotorRight(MotorMinSpeed, MotorMaxSpeed, 1000);
		}
		AVOID_LEFT = 0;
		AVOID_RIGHT = 0;
		AVOID_FRONT = 0;
	}

	else if (State == FORWARD && CheckSecurity() == SECURITY_FAIL) {
		MotorStop();
		TimeToGoHome();			// Check if within working hours, if not, go home
		if (perimeterTracking == 1) {
			cutterOFF();
			perimeterTrackingActive = 1;
			HAL_Delay(500);
			return;
		}
		HAL_Delay(500);

		CheckSecurity();				// Double check status of the sensors when we are standing still

		if (BWF1_Status == OUTSIDE && BWF2_Status == INSIDE) {
			MotorBackward(MotorMinSpeed, MotorMaxSpeed, 400);
			HAL_Delay(500);
			MotorRight(MotorMinSpeed, MotorMaxSpeed, 300 + rnd(800) );
		}
		else if (BWF1_Status == INSIDE && BWF2_Status == OUTSIDE) {
			MotorBackward(MotorMinSpeed, MotorMaxSpeed, 400);
			HAL_Delay(500);
			MotorLeft(MotorMinSpeed, MotorMaxSpeed, 300 + rnd(800) );
		}
		else if (BWF1_Status == OUTSIDE && BWF2_Status == OUTSIDE) {

			Serial_Console("Going Backward\r\n");
			MotorBackward(MotorMinSpeed, MotorMaxSpeed, 3000);
			HAL_Delay(500);
			if (rnd(1000) < 500 ) {
				MotorLeft(MotorMinSpeed, MotorMaxSpeed, 300 + rnd(800) );
			}
			else {
				MotorRight(MotorMinSpeed, MotorMaxSpeed, 300 + rnd(800) );
				}
		}

		HAL_Delay(500);
	}
	else if (State == FORWARD) {

		if (TIM4->CCR2 == 0 && TIM4->CCR3  == 0) {
			State = STOP;
		}
	}
	else if (State == STOP && CheckSecurity() == SECURITY_OK) {
		HAL_Delay(500);
		hold_heading = heading;
		if (cutterStatus == 0 && perimeterTracking == 0) {
			cutterON();
		}
		Serial_Console("Going Forward\r\n");
		MotorForward(MotorMinSpeed, MotorMaxSpeed);

	}
	else if (State == STOP && CheckSecurity() == SECURITY_FAIL) {
		HAL_Delay(500);
		Serial_Console("STOP + Security Fail\r\n");

		CheckSecurity();

		if (BWF1_Status == INSIDE && (BWF2_Status == OUTSIDE || BWF2_Status == NOSIGNAL)) {
			MotorLeft(MotorMinSpeed, MotorMaxSpeed, 300 + rnd(800) );
		}
		else if (BWF2_Status == INSIDE && (BWF1_Status == OUTSIDE || BWF1_Status == NOSIGNAL)) {
			MotorRight(MotorMinSpeed, MotorMaxSpeed, 300 + rnd(800) );
		}
		else if (BWF1_Status == OUTSIDE && BWF2_Status == OUTSIDE) {
			MotorBackward(MotorMinSpeed,MotorMaxSpeed, 3000);
			MotorRight(MotorMinSpeed, MotorMaxSpeed, 300 + rnd(800) );
		}
	}

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_IWDG_Init();
  MX_RNG_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim4);					// Start TIM4 on wheel motors
  HAL_TIM_Base_Start(&htim3);					// Start TIM3 on cutter motor

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);		//M1 Motor PWM
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);		//M1 Motor PWM
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);		//M2 Motor PWM
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);		//M2 Motor PWM
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);		//C1 Motor PWM
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);		//C1 Motor PWM

  HAL_TIM_Base_Start(&htim5);					//Start Timer5 for MicroSeconds delay

  TIM4->CCR1 = 0;								//M1 Motor - Make sure PWM is 0
  TIM4->CCR2 = 0;								//M1 Motor - Make sure PWM is 0
  TIM4->CCR3 = 0;								//M2 Motor - Make sure PWM is 0
  TIM4->CCR4 = 0;								//M2 Motor - Make sure PWM is 0
  TIM3->CCR1 = 0;								//C1 Motor - Make sure PWM is 0
  TIM3->CCR2 = 0;								//C1 Motor - Make sure PWM is 0

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  WatchdogInit();								// STM32 Watchdog - NEVER DISABLE THIS (for safety!)
  HAL_TIM_Base_Start_IT(&htim2);				// 1 second interrupt, will update the watchdog and send info to Serial Console

  Serial_RPi("RazorBoard booting...please wait!\r\n");
  Serial_Console("RazorBoard booting...please wait!\r\n");

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_BUFFER, ADC_SAMPLE_LEN);		// Start the DMA for continues scan mode

  InitFIR();									// Initiate the FIR functions in hardware

  HAL_RNG_Init(&hrng);							// Initiate the True Random Number generator

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);	// Enable interrupt for Serial Console
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);	// Enable interrupt for the Raspberry Pi

  ADC_timer = HAL_GetTick();					// Initial load of the ADC timer
  IMU_timer = HAL_GetTick();					// Initial load of the IMU timer
  MotorSpeedUpdateFreq_timer =  HAL_GetTick();	// Initial load of the MotorSpeedUpdateFreq

  Init6050();									// Start the MPU-6050

  Boundary_Timer = HAL_GetTick();				// Initiate timer for the Boundary Wire

  delay_us(100);

  Serial_RPi("Booting done!\r\n");
  Serial_Console("Booting done!\r\n");

  while (1)
    {

    	// Collect IMU data every 20 ms, non-blocking.
  	if (HAL_GetTick() - IMU_timer >= 20) {
   	  MPU6050_Read_Accel();
  	  MPU6050_Read_Gyro();
  	  ProcessIMUData();
  	  IMU_timer = HAL_GetTick();
  	  }


  	if (perimeterTrackingActive == 0) {

  		CheckSecurity();

  		CheckState();

  		if (State == FORWARD && Force_Active == 1) {
  			if (HAL_GetTick() - MotorSpeedUpdateFreq_timer >= MotorSpeedUpdateFreq) {

  			 UpdateMotorSpeed();
  			 MotorSpeedUpdateFreq_timer =  HAL_GetTick();

  			}
  		}
  		CollectADC();
  		CheckVoltage();
  	}
  	else {
  		perimeterTracker();
  	}

    if (SendInfoStatus == 1) SendInfo();

  	ChargerConnected();

  	HAL_UART_Receive_DMA(&huart1, ConsoleBuffer, CONSOLE_BFR_SIZE);
    HAL_UART_Receive_DMA(&huart2, PIBuffer, PI_BFR_SIZE);

    if (UART1_ready == 1) {
    	parseCommand_Console();
    }
  	if (UART2_ready == 1) {
  		parseCommand_RPI();
  	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8400-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3360-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3360-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

	WatchdogRefresh();		// STM32 Watchdog - NEVER DISABLE THIS (for safety!)
	SendInfoStatus = 1;		// Ready to send Serial Console info

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

}
void HAL_UART_RxCpltCallback_UART1(UART_HandleTypeDef *huart) {
	// UART Rx Complete Callback;
	// Rx Complete is called by: DMA (automatically), if it rolls over
	// and when an IDLE Interrupt occurs
	// DMA Interrupt allays occurs BEFORE the idle interrupt can be fired because
	// idle detection needs at least one UART clock to detect the bus is idle. So
	// in the case, that the transmission length is one full buffer length
	// and the start buffer pointer is at 0, it will be also 0 at the end of the
	// transmission. In this case the DMA rollover will increment the RxRollover
	// variable first and len will not be zero.

	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {					// Check if it is an "Idle Interrupt"
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);								// clear the interrupt
		HAL_UART_DMAStop(&huart1);
		UART1_ready = 1;												// Serial Console data is now ready to be processed

	}

}
void HAL_UART_RxCpltCallback_UART2(UART_HandleTypeDef *huart) {
	// UART Rx Complete Callback;
	// Rx Complete is called by: DMA (automatically), if it rolls over
	// and when an IDLE Interrupt occurs
	// DMA Interrupt allays occurs BEFORE the idle interrupt can be fired because
	// idle detection needs at least one UART clock to detect the bus is idle. So
	// in the case, that the transmission length is one full buffer length
	// and the start buffer pointer is at 0, it will be also 0 at the end of the
	// transmission. In this case the DMA rollover will increment the RxRollover
	// variable first and len will not be zero.

	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {					// Check if it is an "Idle Interrupt"
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);								// clear the interrupt
		HAL_UART_DMAStop(&huart2);
		UART2_ready = 1;												// // Raspberry Pi data is now ready to be processed

	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	// UART Tx Complete Callback;

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
