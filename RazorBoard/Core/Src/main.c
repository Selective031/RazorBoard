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
#include <sram.h>
#include <mpu6050.h>
#include <adc.h>
#include <inttypes.h>
#include <stdbool.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB

IWDG_HandleTypeDef hiwdg;

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
#define SECURITY_BUMPER 5
#define SECURITY_IMU_FAIL 6
#define SECURITY_OUTSIDE 7
#define SECURITY_MOVEMENT 8
#define SECURITY_BACKWARD_OUTSIDE 9

#define INITIAL_MAX_THRESHOLD 10000

#define PI_BFR_SIZE 64                // Buffer size for RPi
#define CONSOLE_BFR_SIZE 64            // Buffer size for Serial Console

#define MOTOR_LEFT_FORWARD TIM4->CCR1
#define MOTOR_LEFT_BACKWARD TIM4->CCR2
#define MOTOR_RIGHT_FORWARD TIM4->CCR4
#define MOTOR_RIGHT_BACKWARD TIM4->CCR3

double Tick1 = 0;
double Tick2 = 0;
double error = 0;
double lastError = 0;
double cumError = 0;
double elapsedTime = 0;
double rateError = 0;
double previousTime = 0;

uint8_t perimeterTracking = 0;
uint8_t perimeterTrackingActive = 0;
uint8_t guideTrackingActive = 0;
uint32_t dockSequenceStartedAt = 0;
uint8_t dockTries = 0;
uint8_t searchingForGuide = 0;
uint8_t guideTrackingOut = 0;
uint32_t lastGuideTrackingLapse = 0;
uint32_t guideTrackingStartedAt = 0;

float32_t magBWFArray1[100] = {0};
float32_t magBWFArray2[100] = {0};
uint8_t magBWFIndex1 = 0;
uint8_t magBWFIndex2 = 0;

float32_t magGuideArray1[100] = {0};
float32_t magGuideArray2[100] = {0};
uint8_t magGuideIndex1 = 0;
uint8_t magGuideIndex2 = 0;
float setPoint = 0;

uint8_t Initial_Start = 0;
uint16_t Start_Threshold = 0;

int bwf1_inside = 0;                // Some stats, how many messages we can detect per second
int bwf2_inside = 0;
int bwf3_inside = 0;
int bwf1_outside = 0;
int bwf2_outside = 0;
int bwf3_outside = 0;

int bwf1guide_inside = 0;
int bwf2guide_inside = 0;
int bwf3guide_inside = 0;
int bwf1guide_outside = 0;
int bwf2guide_outside = 0;
int bwf3guide_outside = 0;

uint32_t Boundary_Timer;            // Keep track of time between signals.
uint32_t Fail_Timer;            // Keep track of time between signals.

float M1_amp = 0.0f;                // Values for AMP per motor
float M2_amp = 0.0f;                // Values for AMP per motor
float C1_amp = 0.0f;                // Values for AMP per motor
float M1_error = 0.0f;              // Auto calibration value stored here.
float M2_error = 0.0f;              // Auto calibration value stored here.
float C1_error = 0.0f;              // Auto calibration value stored here.
float Voltage = 0;

int BWF_cycle = 0;

uint8_t BWF1_Status = 0;
uint8_t BWF2_Status = 0;
uint8_t BWF3_Status = 0;

uint8_t BWF1_guide_status = 0;
uint8_t BWF2_guide_status = 0;
uint8_t BWF3_guide_status = 0;


uint8_t State = STOP;
uint8_t cutterStatus = 0;

uint8_t Collision = 0;
uint8_t IsCollisionChecking = 0;

uint32_t ADC_timer = 0;
uint32_t IMU_timer = 0;
uint32_t OUTSIDE_timer = 0;

uint8_t Docked = 0;
uint8_t Docked_Locked = 0;
uint8_t MasterSwitch = 1;            // This is the "masterswitch", by default this is turned on.
bool isOverturned = false;

uint8_t PIBuffer[PI_BFR_SIZE];
uint8_t ConsoleBuffer[CONSOLE_BFR_SIZE];
uint8_t UART1_ready = 0;
uint8_t UART2_ready = 0;
uint8_t Security = 0;

uint8_t Battery_Ready = 0;            // Battery fully charged?

uint8_t M1_idx = 0;
uint8_t M2_idx = 0;
uint8_t C1_idx = 0;
float M1_force[20];                    // Array of power consumption of Motor M1, used for detecting if we bump into something
float M2_force[20];                    // Array of power consumption of Motor M2, used for detecting if we bump into something
float C1_force[20];
float V1_array[60];
uint8_t V1_index = 0;
float M1_F = 0;
float M2_F = 0;
float C1_F = 0;
uint8_t Force_Active = 0;            // When the mower has gained enough movement, Force_Active will turn on.

float32_t BWF1[ADC_SAMPLE_LEN / 2];
float32_t BWF2[ADC_SAMPLE_LEN / 2];
float32_t BWF3[ADC_SAMPLE_LEN / 2];
arm_fir_instance_f32 S;
uint8_t Signature_Record = FALSE;
uint8_t Guide_Record = FALSE;

float32_t magBWF1, magBWF2, magBWF3;
float32_t Guide_magBWF1, Guide_magBWF2;

uint32_t MotorSpeedUpdateFreq_timer = 0;        // Timer for MotorSpeed Update
uint8_t MotorSpeedUpdateFreq = 100;                // Freq to update motor speed, in milliseconds

uint8_t ChargerConnect = 0;                        // Are we connected to the charger?
uint8_t DEBUG_RAZORBOARD = 0;                    // Used by "debug on/debug off"
uint8_t mag_near_bwf = 0;

uint32_t mag_timer = 0;
uint32_t move_timer = 0;
uint32_t highgrass_timer = 0;
uint32_t tracking_timeout_timer = 0;
uint32_t GoHome_timer_IN = 0;
uint32_t GoHome_timer_OUT = 0;
uint32_t Charger_start_Timer = 0;                // start of charging time
uint16_t Charger_elapsed_Timer = 0;                // how long charge time
uint8_t highgrass_slowdown = 0;
uint16_t highgrass_level = 0;

uint32_t Boundary_timer_IN = 0;
uint32_t Boundary_timer_OUT = 0;

uint8_t bumper_count = 0;
uint8_t move_count = 0;
uint8_t board_revision = 12;

uint32_t BWF_timer = 0;

sram_settings settings;
sram_error errors;
mpu6050 mpu;

arm_pid_instance_f32 PID;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

volatile I2C_HandleTypeDef *razor_hi2c;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_RNG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

static void MotorStop(void);
static void MotorBrake(void);
static void MotorHardBrake(void);
static void MotorForward(uint16_t minSpeed, uint16_t maxSpeed);
static void MotorForwardImpl(uint16_t minSpeed, uint16_t maxSpeed, bool force);
static void MotorBackward(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
static void MotorBackwardImpl(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms, bool forced);
static void MotorLeft(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
static void MotorRight(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
static void MotorLeft_Guide(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
static void MotorRight_Guide(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms);
static bool AbortTurnAndBackIfOutside(uint8_t dir);
static void CheckState(void);
static uint8_t CheckSecurity(void);
static void CheckBWF(void);
static void CheckBWF_Rear(void);
static void CheckBWF_RearGuide(void);
static void cutterON(void);
static void cutterOFF(void);
static void cutterHardBreak(void);
static void parseCommand_RPI(void);
static void parseCommand_Console(void);
static void perimeterTracker(void);
static void ChargerConnected(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
static void delay_us(uint16_t us);
static void ADC_Send(uint8_t channel);
static int ADC_Receive();
static void CollectADC(void);
static void SendInfo(void);
static void CheckMotorCurrent(int RAW);
static void UpdateMotorSpeed();
static void undock(bool force);
static uint32_t rnd(uint32_t maxValue);
static void InitFIR(void);
static void FIR_LEFT(void);
static void FIR_RIGHT(void);
static void FIR_REAR(void);
static void WatchdogRefresh(void);
static void WatchdogInit(void);
static void (*SysMemBootJump)(void);
static void BootLoaderInit(unsigned long BootLoaderStatus);
static void setTime(uint8_t hour, uint8_t minute, uint8_t second);
static void setDate(uint8_t year, uint8_t month, uint8_t day, uint8_t weekday);
static void TimeToGoHome(void);
static void CalcMagnitude(uint8_t idx, uint8_t Sensor);
static void delay(uint32_t time_ms);
static void getIMUOrientation(void);
static void i2c_scanner(void);
static void Serial_DATA(char *msg);
static void scanner(I2C_HandleTypeDef i2c_bus);
static void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef* handle, uint32_t timeout);
static bool wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout);
static void PIDtracker(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
void trackPerimeter();

void checkIfOverGuideWire();

void DockRightSeq();

/* USER CODE BEGIN 0 */

void buzzer(uint8_t count, uint16_t time_between) {

	for (uint8_t x = 0; x < (count +1); x++) {
		HAL_GPIO_TogglePin(GPIOB, buzzer_Pin);
		delay(time_between);
	}

	HAL_GPIO_WritePin(GPIOB, buzzer_Pin, GPIO_PIN_RESET);

}

void CheckChassi(void) {

    if (IsCollisionChecking == 1) {
        return;
    }

    IsCollisionChecking = 1;

    if (State == STOP || MasterSwitch == 0) {
        Serial_Console("Bump!\r\n");

        if (MasterSwitch == 0) {
            return;
        }
    }

    if (Docked == 1) {
        add_error_event("Ignore bump in dock");
        return;
    }

    if (dockSequenceStartedAt > 500) {
        MotorStop();
        add_error_event("Unexpected bump in dock seq");
    }

    if (guideTrackingActive == 1) {
        MotorStop();
        add_error_event("Bump when tracking guide. Checking again...");
        HAL_Delay(2000);
        if (Collision == 1) {
            MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, 1000);
            guideTrackingActive = 0;
            searchingForGuide = true;
        }
    }

//        if (Collision == 1) {
//            if (settings.multi_mode == 2) {
//                DockRightSeq();
//            }
//            if (settings.multi_mode == 1 && Voltage >= settings.Battery_Low_Limit - 1) {
//                add_error_event("Bump again when tracking guide. Moving on.");
//                //MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time);
//                guideTrackingActive = 0;
//                if (guideTrackingOut == 0) {
//                    searchingForGuide = 1;
//                } else {
//                    guideTrackingOut = 0;
//                }
//            } else {
//                add_error_event("Bump again when tracking guide. Disabling.");
//                MasterSwitch = 0;
//            }
//        }
//
//        return;
//    }

	if (State == STOP || State == FORWARD || State == LEFT || State == RIGHT || State == BACKWARD) {
		IsCollisionChecking = 1;

		uint8_t currentState;

	start_state:
		bumper_count++;
		if (bumper_count >= settings.bumper_count_limit) {
			add_error_event("Too many bumber detections - HALT");
			bumper_count = 0;
			MasterSwitch = 0;
			return;
		}
		currentState = State;
        Serial_Console("Bump!\r\n");
		add_error_event("Bumber detection");
		MotorHardBrake();
		delay(500);
		switch (currentState) {

			case BACKWARD:
				MotorForward(settings.motorMinSpeed, settings.motorMaxSpeed);
                HAL_Delay(1000);
                MotorStop();
				if (Collision == 1) goto start_state;
				if (Security == OUTSIDE) return;
				delay(500);
				if (rnd(100000) < 50000) {
					MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, 1200);
					if (Collision == 1) goto start_state;
				}
				else {
					MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, 1200);
					if (Collision == 1) goto start_state;
				}
				break;

            case STOP:
			case FORWARD:
				MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, 1500);
				if (Collision == 1) goto start_state;
				delay(500);
				if (rnd(100000) < 50000) {
					MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, 1200);
					if (Collision == 1) goto start_state;
				}
				else {
					MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, 1200);
					if (Collision == 1) goto start_state;
				}
				break;

			case LEFT:
				MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, 1200);
				if (Collision == 1) goto start_state;
				break;

			case RIGHT:
				MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, 1200);
				if (Collision == 1) goto start_state;
				break;
		}
		bumper_count = 0;
	}
}

void DockRightSeq() {
    if (Docked == 1) {
        return;
    }

    cutterOFF();
    dockTries++;

    Serial_Console("Dock sequence started\r\n");
    add_error_event("Dock sequence started");

    searchingForGuide = 0;
    guideTrackingActive = 0;
    uint32_t rightDuration = settings.lab_1;
    rightDuration *= 10;

    CheckBWF();
    CheckBWF_Rear();

    MotorBackwardImpl(settings.motorMinSpeed, settings.perimeterTrackerSpeed, settings.lab_2, true);
    HAL_Delay(100);

    dockSequenceStartedAt = HAL_GetTick();
    uint16_t speedRight = settings.perimeterTrackerSpeed - 1500;
    uint16_t speedLeft = settings.perimeterTrackerSpeed - 200;
    bool toLeft = false;

    if (BWF1_guide_status == INSIDE) {
        add_error_event("BWF 1 guide in");
    }

    if (BWF3_guide_status == INSIDE) {
        add_error_event("BWF 3 guide in");
    }

    while (HAL_GetTick() - dockSequenceStartedAt < 5000) {
        ChargerConnected();

        if (ChargerConnect == 1 || Docked == 1) {
            MotorBrake();
            add_error_event("Charger connect in seq");
            return;
        }

        if (Collision == 1) {
            MotorBrake();
            add_error_event("Collision in seq");
            MotorBackwardImpl(settings.motorMinSpeed, settings.perimeterTrackerSpeed, 550, true);
            return;
        }

        if (HAL_GetTick() - dockSequenceStartedAt > rightDuration) {
            if (!toLeft) {
                speedRight = settings.perimeterTrackerSpeed - 200;
                speedLeft = settings.perimeterTrackerSpeed - 1500;
            }

            toLeft = true;
        }

        if (!toLeft) {
            speedRight += settings.lab_3;

            if (speedRight > speedLeft) {
                speedRight = speedLeft;
            }
        } else {
            speedLeft += 75;

            if (speedLeft > speedRight) {
                speedLeft = speedRight;
            }
        }

        ChargerConnected();

        MOTOR_LEFT_BACKWARD = 0;
        MOTOR_LEFT_FORWARD = speedLeft;
        MOTOR_RIGHT_FORWARD = speedRight;
        MOTOR_RIGHT_BACKWARD = 0;

        HAL_Delay(50);
    }

    if (ChargerConnect == 0 && Docked == 0) {
        add_error_event("ChargerConnect false after seq");
    }

    MotorBrake();
}

void _DockRightSeq() {
    if (Docked == 1) {
        return;
    }

    cutterOFF();

    Serial_Console("Dock sequence started\r\n");

    searchingForGuide = 0;
    guideTrackingActive = 0;

//    MotorBackwardImpl(settings.motorMinSpeed, settings.motorMaxSpeed, 400, true);
//    HAL_Delay(100);
//    MotorRight_Guide(settings.motorMinSpeed, settings.motorMaxSpeed, 450);
//    HAL_Delay(100);
//    MotorForwardImpl(settings.motorMinSpeed, settings.motorMaxSpeed, true);
//    HAL_Delay(400);
//    MotorStop();
//    MotorLeft_Guide(settings.motorMinSpeed, settings.motorMaxSpeed, 500);
//    HAL_Delay(100);

      MotorBackwardImpl(settings.motorMinSpeed, settings.motorMaxSpeed, 500, true);
      HAL_Delay(100);
      MotorRight_Guide(settings.motorMinSpeed, settings.motorMaxSpeed, settings.BatteryChargeTime); //300

    //getIMUOrientation();
    //MotorForwardImpl(settings.motorMinSpeed * 0.8, settings.motorMaxSpeed * 0.8, true);

    add_error_event("Starting dock seq");
    dockSequenceStartedAt = HAL_GetTick();
    uint16_t speedRight = settings.motorMaxSpeed;
    uint16_t speedLeft = settings.motorMaxSpeed - 200; //200
    while (HAL_GetTick() - dockSequenceStartedAt < 3000) {
        if (ChargerConnect == 1 || Docked == 1) {
            MotorBrake();
            add_error_event("Charger connect in seq");
            return;
        }

        if (Collision == 1) {
            MotorBrake();
            add_error_event("Collision in seq");
            return;
        }

        speedLeft -= 25; //25

        if (speedRight < settings.motorMinSpeed + 100) {
            speedRight = settings.motorMinSpeed + 100;
        }

        if (speedLeft < speedRight) {
            speedLeft = speedRight;
        }

        if (speedLeft < settings.motorMinSpeed + 100) {
            speedLeft = settings.motorMinSpeed + 100;
        }

        ChargerConnected();

        MOTOR_LEFT_BACKWARD = 0;
        MOTOR_LEFT_FORWARD = speedLeft;
        MOTOR_RIGHT_FORWARD = speedRight;
        MOTOR_RIGHT_BACKWARD = 0;

        HAL_Delay(50);
    }

    add_error_event("ChargerConnect false after seq");

    MotorStop();
}

void PIDtracker(void) {

	CheckBWF();

	if (Docked == 1 || MasterSwitch == 0) {
		MotorStop();
        guideTrackingStartedAt = 0; //RM
		return;
	}

	if (guideTrackingStartedAt == 0) {
        guideTrackingStartedAt = HAL_GetTick();
        Boundary_timer_IN = HAL_GetTick();
        Boundary_timer_OUT = HAL_GetTick();
	}

	if (cutterStatus == 1) {
		cutterOFF();
	}

	bool abort = false;

    if (BWF1_Status == OUTSIDE) {
        Boundary_timer_OUT = HAL_GetTick();
        if (guideTrackingOut == 0 && HAL_GetTick() - Boundary_timer_IN >= 6000) {
            add_error_event("Unexpected outside when tracking 1");
            MotorStop();

            if (BWF1_Status == OUTSIDE || BWF2_Status == OUTSIDE) {
                if (BWF1_Status == INSIDE && BWF3_Status == OUTSIDE) {
                    MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time);
                }

                if (BWF1_Status == OUTSIDE && BWF2_Status == OUTSIDE) {
                    MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time);
                }

                CheckBWF();

                if (BWF1_Status == INSIDE && BWF2_Status == INSIDE) {
                    MotorForward(settings.motorMinSpeed, settings.motorMaxSpeed);
                    HAL_Delay(1500);
                    MotorStop();
                }
            }
            abort = true;
        }
    }

    if (BWF2_Status == INSIDE) {
        Boundary_timer_IN = HAL_GetTick();
    }

    if (guideTrackingOut == 1 && lastGuideTrackingLapse > 160000) {
        lastGuideTrackingLapse = 160000;
    }

    if (guideTrackingOut == 1 && HAL_GetTick() - guideTrackingStartedAt > lastGuideTrackingLapse) {
        abort = true;
        Serial_Console("[DOCK FREE]\r\n");
    }

    if ((BWF1_Status == OUTSIDE || BWF2_Status == OUTSIDE) && guideTrackingOut == 1) {
        MotorBackwardImpl(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorBackward_time, false);
        abort = true;
    }

    if ((BWF1_Status == OUTSIDE || BWF2_Status == OUTSIDE) && HAL_GetTick() - guideTrackingStartedAt < 5000 && guideTrackingOut == 0) {
        MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time);
        add_error_event("Unexpected outside when tracking 2");
        abort = true;
    }

    if (abort) {
        MotorStop();
        cutterON();

        lastGuideTrackingLapse = 0;
        guideTrackingStartedAt = 0;
        guideTrackingActive = 0;
        searchingForGuide = 0;
        guideTrackingOut = 0;
        return;
    }

    if (BWF1_guide_status == OUTSIDE) {
        GoHome_timer_OUT = HAL_GetTick();
        if (guideTrackingOut == 0 && HAL_GetTick() - GoHome_timer_IN >= 15000) {
            guideTrackingActive = 0;
            searchingForGuide = 1;
            guideTrackingStartedAt = 0;
            MotorStop();
            add_error_event("Stuck guidetracking BWF1 OUT - retry");
            return;
        }
    }

    if (BWF1_guide_status == INSIDE) {
        GoHome_timer_IN = HAL_GetTick();
        if (guideTrackingOut == 0 && HAL_GetTick() - GoHome_timer_OUT >= 15000) {
            guideTrackingActive = 0;
            searchingForGuide = 1;
            guideTrackingStartedAt = 0;
            MotorStop();
            add_error_event("Stuck guidetracking BWF1 IN - retry");
            return;
        }
    }

	float PIDerror = 0;

	//if (settings.move_count_limit == 71)
    {
        PID.Kp = settings.kp;
        PID.Ki = settings.ki;
        PID.Kd = settings.kd;
	}

	int speed_M1 = settings.perimeterTrackerSpeed;
	int speed_M2 = settings.perimeterTrackerSpeed;

    bool closeToDock = magBWF1 > Guide_magBWF1 || magBWF2 > Guide_magBWF2 || magBWF1 >= settings.magMinValue || magBWF2 >= settings.magMinValue || magBWF1 > 160;
	if ((BWF1_Status == OUTSIDE && BWF2_Status == OUTSIDE) || closeToDock) {
	    speed_M1 = (settings.motorMinSpeed + settings.perimeterTrackerSpeed) / 2;
	    speed_M2 = (settings.motorMinSpeed + settings.perimeterTrackerSpeed) / 2;

        //PID.Kp = 20;
        //PID.Ki = 0;
        //PID.Kd = 10;

        if (settings.move_count_limit != 72) {
            PID.Kp = 50;
            PID.Ki = 0;
            PID.Kd = 0.05; //2?
        }
        // NEW:
        //PID.Kp = 25;
        //PID.Ki = 0.05;
        //PID.Kd = 0.02;

        if (Collision == 1) {
            MotorStop();
        }
	}

    if ((BWF1_Status == OUTSIDE && BWF2_Status == OUTSIDE)) {
        MotorStop();
        HAL_Delay(50);
        CheckBWF();
        CheckBWF_Rear();
        if ((BWF1_Status == OUTSIDE && BWF2_Status == OUTSIDE && BWF3_Status == INSIDE)) {
            DockRightSeq();
        }
        return;
    }

    uint8_t leftSideTrigger = guideTrackingOut == 0 ? INSIDE : OUTSIDE;
    uint8_t rightSideTrigger = guideTrackingOut == 0 ? OUTSIDE : INSIDE;

    CheckBWF();
	if (BWF1_guide_status == leftSideTrigger) {
		if (setPoint == -20) arm_pid_init_f32(&PID, 1);
		setPoint = 20;
		PIDerror = arm_pid_f32(&PID, setPoint);
	}

	if (BWF1_guide_status == rightSideTrigger) {
		if (setPoint == 20) arm_pid_init_f32(&PID, 1);
		setPoint = -20;
		PIDerror = arm_pid_f32(&PID, setPoint);
	}

	if (PIDerror < 0) speed_M1 -= abs((int)PIDerror);
	if (PIDerror > 0) speed_M1 += abs((int)PIDerror);

	if (PIDerror < 0) speed_M2 += abs((int)PIDerror);
	if (PIDerror > 0) speed_M2 -= abs((int)PIDerror);

	if (speed_M1 < 1800) speed_M1 = 1800;
	if (speed_M2 < 1800) speed_M2 = 1800;

	if (speed_M1 > 3359) speed_M1 = 3359;
	if (speed_M2 > 3359) speed_M2 = 3359;

	MOTOR_LEFT_BACKWARD = 0;
	MOTOR_LEFT_FORWARD = speed_M1;
	MOTOR_RIGHT_BACKWARD = 0;
	MOTOR_RIGHT_FORWARD = speed_M2;
}

bool wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout)
 {
    uint32_t Tickstart = HAL_GetTick();
    bool ret = true;
    /* Wait until flag is set */
    for(;(state != HAL_GPIO_ReadPin(port, pin)) && (true == ret);)
    {
        /* Check for the timeout */
        if (timeout != HAL_MAX_DELAY)
        {
            if ((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout))
            {
                ret = false;
            }
            else
            {
            }
        }
        asm("nop");
    }
    return ret;
}

void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef* handle, uint32_t timeout)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. Clear PE bit.
    CLEAR_BIT(handle->Instance->CR1, I2C_CR1_PE);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_I2C_DeInit(handle);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = I2C1_SCL_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = I2C1_SDA_Pin;
    HAL_GPIO_Init(I2C1_SDA_GPIO_Port, &GPIO_InitStructure);

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET, timeout);
    wait_for_gpio_state_timeout(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET, timeout);

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_RESET);

    // 5. Check SDA Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_RESET, timeout);

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_RESET);

    // 7. Check SCL Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_RESET, timeout);

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET, timeout);

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET, timeout);

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;

    GPIO_InitStructure.Pin = I2C1_SCL_Pin;
    HAL_GPIO_Init(I2C1_SCL_GPIO_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = I2C1_SDA_Pin;
    HAL_GPIO_Init(I2C1_SDA_GPIO_Port, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
    CLEAR_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
    SET_BIT(handle->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    // Call initialization function.
    HAL_I2C_Init(handle);
}

void Serial_DATA(char *msg) {

	HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), 100);

}
void scanner(I2C_HandleTypeDef i2c_bus) {
	Serial_Console("Scanning I2C Bus...\r\n");
	uint8_t i = 0, ret;
	for(i=1; i<128; i++)
    {
    	ret = HAL_I2C_IsDeviceReady(&i2c_bus, (uint16_t)(i<<1), 3, 10);
    	if (ret != HAL_OK) /* No ACK Received At That Address */
    	{

        }
    	else if(ret == HAL_OK)
    	{
    	    sprintf(msg, "0x%X \r\n", i);
    	    HAL_UART_Transmit(&huart1, (uint8_t *)msg, sizeof(msg), 10000);
    	}
    }
	Serial_Console("Scanning Done.\r\n");
}
void i2c_scanner(void) {

	//72 - ADC
	//104 - MPU6050

	if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(72<<1), 10, 100) == HAL_OK &&
			HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(104<<1), 10, 100) == HAL_OK) {
		Serial_Console("Scanner OK PCB 1.2\r\n");
		board_revision = 12;
		razor_hi2c = &hi2c1;

	} else if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(104<<1), 10, 100) == HAL_OK){
		Serial_Console("Scanner OK PCB 1.0\r\n");
		board_revision = 10;
		razor_hi2c = &hi2c2;
	}
	else {
		board_revision = 12;
		razor_hi2c = &hi2c1;
	}
}

void getIMUOrientation(void) {
	for (uint8_t x = 0; x < 20; x++) {
		MPU6050_Read_Accel();
		MPU6050_Read_Gyro();
		ProcessIMUData(settings);
	}
}

void delay(uint32_t time_ms) {
	uint32_t timer;
	timer = HAL_GetTick();

    CheckSecurity();
	while (HAL_GetTick() - timer < time_ms) {
		CheckSecurity();
	}
}

void reInitIMU(void) {
	// If the I2C bus hangs, this will clear the deadlock and re-init the MPU

	MotorStop();
	cutterOFF();

	add_error_event("reInit IMU");
	Serial_Console("reInit IMU\r\n");

	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	Init6050();
}

void CalcMagnitude(uint8_t idx, uint8_t Sensor) {
	float32_t Mag_Out[LENGTH_SAMPLES / 4];
	float32_t sum = 0;
	float32_t magValue;
	float32_t BWF[128];
	uint16_t count = 0;

	for (uint16_t x = idx; x < (idx + 128) - 1; x++ ) {
		if (Sensor == 1 || Sensor == 11) {
			BWF[count] = BWF1[x];
			count++;
		}
		if (Sensor == 2 || Sensor == 12) {
			BWF[count] = BWF2[x];
			count++;
		}

	}

	if (Sensor == 1 || Sensor == 11) {
		arm_cmplx_mag_f32(BWF, Mag_Out, LENGTH_SAMPLES / 4);
	} else if (Sensor == 2 || Sensor == 12) {
		arm_cmplx_mag_f32(BWF, Mag_Out, LENGTH_SAMPLES / 4);
	}

	for (int y = 0; y < (LENGTH_SAMPLES / 4); y++) {
		sum += Mag_Out[y];
	}
	arm_sqrt_f32(sum, &magValue);

	if (Sensor == 1) {
		magBWFArray1[magBWFIndex1] = magValue;
		magBWFIndex1++;
		if (magBWFIndex1 == 100) magBWFIndex1 = 0;
		if (magBWFArray1[99] != 0) arm_mean_f32(magBWFArray1, 100, &magBWF1);
	} else if (Sensor == 2) {
		magBWFArray2[magBWFIndex2] = magValue;
		magBWFIndex2++;
		if (magBWFIndex2 == 100) magBWFIndex2 = 0;
		if (magBWFArray2[99] != 0) arm_mean_f32(magBWFArray2, 100, &magBWF2);
	}
	if (Sensor == 11) {
		magGuideArray1[magGuideIndex1] = magValue;
		magGuideIndex1++;
		if (magGuideIndex1 == 100) magGuideIndex1 = 0;
		if (magGuideArray1[99] != 0) arm_mean_f32(magGuideArray1, 100, &Guide_magBWF1);
	} else if (Sensor == 12) {
		magGuideArray2[magGuideIndex2] = magValue;
		magGuideIndex2++;
		if (magGuideIndex2 == 100) magGuideIndex2 = 0;
		if (magGuideArray2[99] != 0) arm_mean_f32(magGuideArray2, 100, &Guide_magBWF2);
	}

    if (!guideTrackingActive) {
        if ((magBWF1 >= settings.magValue || magBWF2 >= settings.magValue) && (MOTOR_LEFT_FORWARD == settings.motorMaxSpeed || MOTOR_RIGHT_FORWARD == settings.motorMaxSpeed) && Docked == 0) {
            if (mag_near_bwf == 0) {
                mag_near_bwf = 1;
                highgrass_slowdown = 0;
                sprintf(msg, "proximity-alert BWF1: %f BWF2: %f\r\n", magBWF1, magBWF2);
                Serial_Console(msg);
            }
            mag_timer = HAL_GetTick();
        } else if (magBWF1 <= settings.magMinValue && magBWF2 <= settings.magMinValue && mag_near_bwf == 1 && Docked == 0) {
            if (HAL_GetTick() - mag_timer >= 3000) {
                mag_near_bwf = 0;
                for (uint32_t x = (settings.motorMaxSpeed * settings.proximitySpeed); x < settings.motorMaxSpeed; x++) {
                    MOTOR_LEFT_FORWARD = x;
                    MOTOR_RIGHT_FORWARD = x;
                    HAL_Delay(1);
                    if (CheckSecurity() != SECURITY_OK) {
                        Serial_Console("Security fail while in proximity loop\r\n");
                        add_error_event("Security fail while in proximity loop");
                        break;
                    }
                }
            }
        }
    }
}

void TimeToGoHome(void) {
	// Get Time and check if we should go home

	RTC_TimeTypeDef currTime = {0};
	RTC_DateTypeDef currDate = {0};

	HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);

	if (currTime.Hours >= settings.WorkingHourEnd || Voltage <= settings.Battery_Low_Limit) {

        if (settings.multi_mode == 1) {
            Serial_Console("[REQUEST TO DOCK]");

            if (Voltage < settings.Battery_Low_Limit - 1) {
                add_error_event("Low voltage");
            }
        } else {
            sprintf(emsg, "tracking enabled %d", currTime.Hours);
            add_error_event(emsg);
            if (settings.use_guide_wire == 0) {
                perimeterTracking = 1;
            } else {
                searchingForGuide = 1;
            }
        }
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
	if (BootLoaderStatus == 1) {
		HAL_DeInit();
		SysTick->CTRL = 0;
		SysTick->LOAD = 0;
		SysTick->VAL = 0;
		__set_PRIMASK(1);
		__set_MSP(0x20001000);
		SysMemBootJump();
		while (1);
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
	arm_fir_init_f32(&S, NUM_TAPS, (float32_t *) &firCoeffs32[0], &firStateF32[0], blocksize);
}

void FIR_LEFT(void) {
	uint32_t i;
	uint32_t blockSize = BLOCK_SIZE;
	uint32_t numBlocks = LENGTH_SAMPLES / BLOCK_SIZE;
	float32_t *inputF32, *outputF32;
	inputF32 = &BWF1[0];
	outputF32 = &Output[0];
	for (i = 0; i < numBlocks; i++) {
		arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
	}
	for (int x = 0; x < 256; x++) {
		BWF1[x] = (float) outputF32[x];
	}
}

void FIR_RIGHT(void) {
	uint32_t i;
	uint32_t blockSize = BLOCK_SIZE;
	uint32_t numBlocks = LENGTH_SAMPLES / BLOCK_SIZE;
	float32_t *inputF32, *outputF32;
	inputF32 = &BWF2[0];
	outputF32 = &Output[0];
	for (i = 0; i < numBlocks; i++) {
		arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
	}
	for (int x = 0; x < 256; x++) {
		BWF2[x] = (float) outputF32[x];
	}
}

void FIR_REAR(void) {
	uint32_t i;
	uint32_t blockSize = BLOCK_SIZE;
	uint32_t numBlocks = LENGTH_SAMPLES / BLOCK_SIZE;
	float32_t *inputF32, *outputF32;
	inputF32 = &BWF3[0];
	outputF32 = &Output[0];
	for (i = 0; i < numBlocks; i++) {
		arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
	}
	for (int x = 0; x < 256; x++) {
		BWF3[x] = (float) outputF32[x];
	}
}

uint32_t rnd(uint32_t maxValue) {
	// Our random number generator

	uint32_t rndnum;
	rndnum = HAL_RNG_GetRandomNumber(&hrng) % maxValue;
	return rndnum + 1;
}

void CheckMotorCurrent(int RAW) {
	// Check if any motor is experiencing a spike in power, then we probably hit something.
	float M1, M2;
	if (M1_idx == 10 || M2_idx == 10 || C1_idx == 10) {
		Force_Active = 1;
	}

	if (Channel == M1_addr) {
		M1_Value = RAW;
		M1 = fabs(((M1_Value * 0.1875) - 2500) / 100);
		if (Initial_Start == 0) {
			M1_error = M1;
		}
		M1_amp = fabs(M1 - M1_error);
		M1_force[M1_idx] = M1_amp;
		M1_idx++;
		if (M1_idx == 20) {
			M1_idx = 0;
		}
		float ForceM1 = 0;
		for (int x = 0; x < 20; x++) {
			ForceM1 += M1_force[x];
		}
		M1_F = ForceM1 / 20;
		if (M1_F < settings.Motor_Min_Limit) {
			M1_F = settings.Motor_Min_Limit;
		}
		if ((M1_amp >= settings.Motor_Max_Limit || M1_amp >= M1_F * settings.Motor_Limit) && State == (FORWARD || RIGHT) && Force_Active == 1 && M1_amp < 10.0 && perimeterTrackingActive == 0 && guideTrackingActive == 0) {
			sprintf(emsg, "M1 current: %.2f, %.2f", M1_amp, M1_F);
			add_error_event(emsg);
			sprintf(msg, "Motor Current Limit reached for M1: %.2f", M1_amp);
			Serial_Console(msg);

			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {
				return;
			}
			MotorBrake();
			delay(500);
			MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, 1500);
			delay(500);
			MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, 1000);
			Force_Active = 0;
			bumper_count++;
			M1_amp = 0;
		}
	} else if (Channel == M2_addr) {
		M2_Value = RAW;
		M2 = fabs(((M2_Value * 0.1875) - 2500) / 100);
		if (Initial_Start == 0) {
			M2_error = M2;
		}
		M2_amp = fabs(M2 - M2_error);
		M2_force[M2_idx] = M2_amp;
		M2_idx++;
		if (M2_idx == 20) {
			M2_idx = 0;
		}
		float ForceM2 = 0;
		for (int x = 0; x < 20; x++) {
			ForceM2 += M2_force[x];
		}
		M2_F = ForceM2 / 20;
		if (M2_F < settings.Motor_Min_Limit) {
			M2_F = settings.Motor_Min_Limit;
		}
		if ((M2_amp >= settings.Motor_Max_Limit || M2_amp >= M2_F * settings.Motor_Limit) && State == (FORWARD || LEFT) && Force_Active == 1 && M2_amp < 10.0 && perimeterTrackingActive == 0 && guideTrackingActive == 0) {
			sprintf(emsg, "M2 current: %.2f, %.2f", M2_amp, M2_F);
			add_error_event(emsg);
			sprintf(msg, "Motor Current Limit reached for M2: %.2f", M2_amp);
			Serial_Console(msg);

			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {
				return;
			}
			MotorBrake();
			delay(500);
			MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, 1500);
			delay(500);
			MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, 1000);
			Force_Active = 0;
			bumper_count++;
			M2_amp = 0;
		}
	}
}

void SendInfo() {
	if (DEBUG_RAZORBOARD == 0) {
		bwf1_inside = 0;
		bwf1_outside = 0;
		bwf2_inside = 0;
		bwf2_outside = 0;
		bwf3_inside = 0;
		bwf3_outside = 0;

		bwf1guide_inside = 0;
		bwf1guide_outside = 0;
        bwf2guide_inside = 0;
        bwf2guide_outside = 0;
        bwf3guide_inside = 0;
        bwf3guide_outside = 0;

		return;
	}

	// Send info to the Console & Raspberry PI - Update frequency is ~1 Hz
	RTC_TimeTypeDef currTime = {0};
	RTC_DateTypeDef currDate = {0};

	HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);

	sprintf(msg, "(D) M1: %.2f\r\n", M1_amp);
	Serial_DATA(msg);
	sprintf(msg, "(D) M2: %.2f\r\n", M2_amp);
	Serial_DATA(msg);
	sprintf(msg, "(D) C1: %.2f\r\n", C1_amp);
	Serial_DATA(msg);
	sprintf(msg, "(D) V1: %.2f\r\n", Voltage);
	Serial_DATA(msg);
	sprintf(msg, "(D) Charger Connected: %d\r\n", ChargerConnect);
	Serial_DATA(msg);
	if (ChargerConnect == 1) {
		sprintf(msg, "(D) Charger elapsed time (min): %d\r\n", Charger_elapsed_Timer);
		Serial_DATA(msg);
	}
	sprintf(msg, "(D) IN-> BWF1: %d BWF2: %d BWF3: %d\r\n(D) OUT-> BWF1: %d BWF2: %d BWF3: %d\r\n", bwf1_inside, bwf2_inside,
			bwf3_inside, bwf1_outside, bwf2_outside, bwf3_outside);
	Serial_DATA(msg);

	sprintf(msg, "(D) IN_GUIDE-> BWF1: %d BWF2: %d BWF3: %d\r\n(D) OUT_GUIDE-> BWF1: %d BWF2: %d BWF3: %d\r\n", bwf1guide_inside, bwf2guide_inside, bwf3guide_inside,
			bwf1guide_outside, bwf2guide_outside, bwf3guide_outside);
	Serial_DATA(msg);

	sprintf(msg, "(D) Magnitude -> BWF1: %.2f BWF2: %.2f\r\n", magBWF1, magBWF2);
	Serial_DATA(msg);
	sprintf(msg, "(D) GuideMgmt -> BWF1: %.2f BWF2: %.2f\r\n", Guide_magBWF1, Guide_magBWF2);
	Serial_DATA(msg);
	sprintf(msg, "(D) Battery Fully Charged: %d\r\n", Battery_Ready);
	Serial_DATA(msg);
    sprintf(msg, "(D) Docked: %d\r\n", Docked);
    Serial_Console(msg);
    sprintf(msg, "(D) Enabled: %d\r\n", MasterSwitch);
    Serial_Console(msg);
	sprintf(msg, "(D) Roll: %.2f Pitch: %2.f Yaw: %2.f\r\n", mpu.roll, mpu.pitch, mpu.yaw);
	Serial_DATA(msg);
	sprintf(msg, "(D) Time: %d:%d:%d\r\n", currTime.Hours, currTime.Minutes, currTime.Seconds);
	Serial_DATA(msg);
	sprintf(msg, "(D) Date: 20%d-%d-%d\r\n", currDate.Year, currDate.Month, currDate.Date);
	Serial_DATA(msg);
	sprintf(msg, "(D) Movement: %.2f\r\n", mpu.movement);
	Serial_DATA(msg);
	sprintf(msg, "(D) Perimeter tracking: %d\r\n", perimeterTracking);
	Serial_DATA(msg);
    sprintf(msg, "(D) Guide tracking: %d\r\n", searchingForGuide);
    Serial_DATA(msg);
    sprintf(msg, "(D) Guide tracking active: %d\r\n", guideTrackingActive);
    Serial_DATA(msg);
	if (mpu.movement < settings.movement) {
		sprintf(msg, "(D) Movement Verdict: Standing\r\n");
	} else {
		sprintf(msg, "(D) Movement Verdict: Moving\r\n");
	}
	Serial_DATA(msg);
	if (Security == 0) {
		Serial_DATA("(D) Security FAIL\r\n");
	}
	if (Security == 1) {
		Serial_DATA("(D) Security OK\r\n");
	}
	if (Security == 2) {
		Serial_DATA("(D) Security OUTSIDE\r\n");
	}
	if (Security == 3) {
		Serial_DATA("(D) Security LEFT\r\n");
	}
	if (Security == 4) {
		Serial_DATA("(D) Security RIGHT\r\n");
	}
	if (Security == 5) {
		Serial_DATA("(D) Security BUMPER\r\n");
	}
	if (Security == 6) {
		Serial_DATA("(D) Security IMU_FAIL\r\n");
	}
	if (Security == 7) {
		Serial_DATA("(D) Security OUSIDE\r\n");
	}
	if (Security == 8) {
		Serial_DATA("(D) Security MOVEMENT\r\n");
	}
	if (Security == 9) {
		Serial_DATA("(D) Security BACKWARD_OUTSIDE\r\n");
	}
	if (State == 0) {
		Serial_DATA("(D) State STOP\r\n");
	}
	if (State == 1) {
		Serial_DATA("(D) State FORWARD\r\n");
	}
	if (State == 2) {
		Serial_DATA("(D) State BACKWARD\r\n");
	}
	if (State == 3) {
		Serial_DATA("(D) State LEFT\r\n");
	}
	if (State == 4) {
		Serial_DATA("(D) State RIGHT\r\n");
	}
	if (State == 5) {
		Serial_DATA("(D) State AVOID_OBSTACLE\r\n");
	}
	if (State == 6) {
		Serial_DATA("(D) State FAIL\r\n");
	}
	if (State == 7) {
		Serial_DATA("(D) State BRAKE\r\n");
	}
	if (State == 8) {
		Serial_DATA("(D) State HARDBRAKE\r\n");
	}
	sprintf(msg, "(D) Docking Station Locked: %u\r\n", Docked_Locked);
	Serial_Console(msg);
	sprintf(msg, "(D) Board_Revision: %d\r\n", board_revision);
	Serial_DATA(msg);
	sprintf(msg, "(D) BWF Cycle Time: %d\r\n", BWF_cycle);
	Serial_DATA(msg);

	/*
    char Data[128];

    sprintf(Data, "Battery Voltage,%.2f\r\n"
                  "M1 Current,%.2f\r\n"
                  "M2 Current,%.2f\r\n"
                  "C1 Current,%.2f\r\n"
                  "Security,%d\r\n"
                  "Mower State,%d\r\n", Voltage, M1_amp, M2_amp, C1_amp, Security, State);

	 */
	bwf1_inside = 0;
	bwf1_outside = 0;
	bwf2_inside = 0;
	bwf2_outside = 0;
	bwf3_inside = 0;
	bwf3_outside = 0;

	bwf1guide_inside = 0;
	bwf1guide_outside = 0;
    bwf2guide_inside = 0;
    bwf2guide_outside = 0;
    bwf3guide_inside = 0;
    bwf3guide_outside = 0;

	//    HAL_UART_Transmit(&huart2, (uint8_t *) &Data, strlen(Data), 100);

}

void CollectADC() {
	// Collect IC2 data from the external ADC, in a non-blocking way.

	if (Channel == M1_addr && Channel_Status == 0) {
		ADC_Send(Channel);
		Channel_Status = 1;
		ADC_timer = HAL_GetTick();
	} else if (Channel == M2_addr && Channel_Status == 0) {
		ADC_Send(Channel);
		Channel_Status = 1;
		ADC_timer = HAL_GetTick();
	} else if (Channel == C1_addr && Channel_Status == 0) {
		ADC_Send(Channel);
		Channel_Status = 1;
		ADC_timer = HAL_GetTick();
	} else if (Channel == V1_addr && Channel_Status == 0) {
		ADC_Send(Channel);
		Channel_Status = 1;
		ADC_timer = HAL_GetTick();
	}

	if ((HAL_GetTick() - ADC_timer) >= 20 && Channel_Status == 1) {
		int RAW = 0;
		RAW = ADC_Receive();

		CheckMotorCurrent(RAW);

		if (Channel == C1_addr && Docked == 0) {
			C1_Value = RAW;
			float C1;
			C1 = fabs(((C1_Value * 0.1875) - 2500) / 100);
			if (Initial_Start == 0) {
				C1_error = C1;
			}
			C1_amp = fabs(C1 - C1_error);

			if (C1_amp >= settings.highgrass_Limit && mag_near_bwf == 0 && C1_amp < 10.0 && perimeterTrackingActive == 0 && guideTrackingActive == 0 && searchingForGuide == 0) {
                bool recentSlowdown = HAL_GetTick() - highgrass_timer <= 7000;
                if (highgrass_slowdown == 1 || recentSlowdown) {
                    highgrass_level += 100;
                    add_error_event("Grasslevel increased");
                } else {
                    add_error_event("High Grass detected");
                    highgrass_level = 100;
                }

                if (highgrass_level > 600) {
                    add_error_event("Grasslevel > 6");
                    cutterOFF();
                    highgrass_level = 0;
                }

				highgrass_slowdown = 1;
				MotorStop();
				MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, 1500);
				delay(2500);
				MotorForward(settings.motorMinSpeed, settings.motorCruiseSpeed * 0.8 - highgrass_level);
				highgrass_timer = HAL_GetTick();
			} else {
				if (HAL_GetTick() - highgrass_timer >= 5000 && highgrass_slowdown == 1 && MasterSwitch == 1 && Docked == 0) {
					highgrass_slowdown = 0;
					for (uint32_t x = (settings.motorCruiseSpeed * 0.8 - highgrass_level); x < settings.motorCruiseSpeed; x++) {
						MOTOR_LEFT_FORWARD = x;
						MOTOR_RIGHT_FORWARD = x;
						delay(1);
						if (CheckSecurity() != SECURITY_OK) {
							add_error_event("Security fail while in high grass loop");
							break;
						}
					}
				}
			}

			if (C1_amp >= settings.Cutter_Limit && C1_amp < 10.0) {
				sprintf(emsg, "C1 current: %.2f", C1_amp);
				add_error_event(emsg);
				MasterSwitch = 0;
				return;
			}
		} else if (Channel == V1_addr) {
			V1_Value = RAW;
			V1_array[V1_index] = (V1_Value * 0.1875) * settings.voltageMultiply / 1000;
			V1_index++;
			if (V1_index >= 60) {
				V1_index = 0;
			}
			float volt_sum = 0.0;
			for (uint8_t x = 0; x < 60; x++) {
				volt_sum += V1_array[x];
			}
			Voltage = volt_sum / 60;
		}

		if (Channel == M1_addr) {
			Channel = M2_addr;
		} else if (Channel == M2_addr) {
			Channel = C1_addr;
		} else if (Channel == C1_addr) {
			Channel = V1_addr;
		} else if (Channel == V1_addr) {
			Channel = M1_addr;
		}
		Channel_Status = 0;
	}
}

void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim5, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim5) < us);  // wait for the counter to reach the us input in the parameter
}

void undock(bool force) {
	// Simple undock sequence,  check if Battery is ready (fully charged) and if we are within working hours.

	RTC_TimeTypeDef currTime = {0};
	RTC_DateTypeDef currDate = {0};

	HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);

	if ((currTime.Hours >= settings.WorkingHourStart && currTime.Hours < settings.WorkingHourEnd && Battery_Ready == 1 && Docked == 1 && Docked_Locked == 0) || force) {

	    if (ChargerConnect == 1) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
            add_error_event("Charger disconnect");
            Serial_Console("Charger disconnected.\r\n");
            ChargerConnect = 0;
        }

		add_error_event("Switch Main Battery");
		Serial_Console("Switching to Main Battery\r\n");
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
		delay(5000);

		MasterSwitch = 1;
		Docked = 0;

		mpu.roll = 0;
		mpu.pitch = 0;
		mpu.yaw = 0;
		setPoint = 0;

		Initial_Start = 0;
		Start_Threshold = 0;
		Battery_Ready = 0;
		lastError = 0;
		perimeterTracking = 0;
		perimeterTrackingActive = 0;
        searchingForGuide = 0;
		guideTrackingActive = 0;
		write_all_settings(settings);

		MotorBackwardImpl(settings.motorMinSpeed, settings.motorMaxSpeed, settings.undock_backing_seconds * 1000, true);

        if (lastGuideTrackingLapse > 0) {
            lastGuideTrackingLapse -= settings.undock_backing_seconds * 1000;
            searchingForGuide = 1;
            guideTrackingActive = 1;
            guideTrackingOut = 1;
            dockSequenceStartedAt = 0;
            Fail_Timer = 0;
            guideTrackingStartedAt = HAL_GetTick();
            MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, 1550);            // This needs to be changed if your docking is on the right side
            Fail_Timer = 0;
        } else {
            MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, 800);            // This needs to be changed if your docking is on the right side
            Serial_Console("[DOCK FREE]\r\n");
        }
	}
}

void ChargerConnected(void) {

	// Is the charger connected?

	if (ChargerConnect == 0) {                    // We are not charging, reset charger start timer
		Charger_start_Timer = HAL_GetTick();
		Charger_elapsed_Timer = 0;
	}

	if (ChargerConnect == 1) {                    // We are charging, calculate duration in minutes
		Charger_elapsed_Timer = (HAL_GetTick() - Charger_start_Timer) / 60000;
        dockSequenceStartedAt = 0;
	}
	if (ChargerConnect == 1 || Docked == 1) {
		if (Voltage >= settings.Battery_High_Limit && Battery_Ready == 0 && Charger_elapsed_Timer >= settings.BatteryChargeTime) {
			Battery_Ready = 1;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
			add_error_event("Charger disconnect");
            Serial_Console("Charger disconnected.\r\n");
            Serial_Console("[FULLY CHARGED]\r\n");
			ChargerConnect = 0;
			delay(5000);
			undock(false);
		}
		return;
	}

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {            // Read Volt sense pin

	    if (guideTrackingActive == 1 && guideTrackingStartedAt > 0) {
            lastGuideTrackingLapse = HAL_GetTick() - guideTrackingStartedAt;

            if (lastGuideTrackingLapse < (settings.undock_backing_seconds + 3) * 1000) {
                lastGuideTrackingLapse = 0;
            }
	    }

		HAL_Delay(settings.HoldChargeDetection);                          // Wait for a while so a proper connection is made
		Force_Active = 0;
		MotorBrake();
		cutterOFF();
        ChargerConnect = 1;
        Docked = 1;
        dockTries = 0;
        guideTrackingActive = 0;
        searchingForGuide = 0;
        perimeterTrackingActive = 0;
		add_error_event("Charger connect");
		Serial_Console("Charger Connected\r\n");
		delay(5000);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);               // Main Power switch
		Serial_Console("Changing Main Power\r\n");
		delay(2000);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);              // Charger Switch
		add_error_event("Charging active");
		Serial_Console("Charging activated\r\n");

		return;
	}
}

void perimeterTracker(void) {
	if (Docked == 1) return;
	CheckSecurity();

	mag_near_bwf = 0;
	highgrass_slowdown = 0;

	elapsedTime = HAL_GetTick() - previousTime;

	if (BWF2_Status == OUTSIDE) {
		Tick1 -= elapsedTime;
		Tick2 = 0;
		GoHome_timer_IN = HAL_GetTick();

		if (HAL_GetTick() - GoHome_timer_OUT >= 10000) {        // 10 seconds
			perimeterTrackingActive = 0;
			perimeterTracking = 0;
			MasterSwitch = 0;
			add_error_event("Stuck perimeterTracking BWF2 OUT - HALT");
			return;
		}
	}

	if (BWF2_Status == INSIDE) {
		Tick2 -= (elapsedTime * 4);
		Tick1 = 0;
		GoHome_timer_OUT = HAL_GetTick();

		if (HAL_GetTick() - GoHome_timer_IN >= 10000) {            // 10 seconds
			perimeterTrackingActive = 0;
			perimeterTracking = 0;
			MasterSwitch = 0;
			add_error_event("Stuck perimeterTracking BWF2 IN - HALT");
			return;
		}
	}

	error = settings.perimeterTrackerSpeed - (Tick1 + Tick2);                // determine error
	cumError += error * elapsedTime;               // compute integral
	rateError = (error - lastError) / elapsedTime;   // compute derivative

	double out = settings.kp * error + settings.ki * cumError + settings.kd * rateError;                //PID output

	lastError = error;                             //remember current error
	previousTime = HAL_GetTick();                  //remember current time

	int speedA = (settings.perimeterTrackerSpeed + round(out));
	int speedB = (settings.perimeterTrackerSpeed - round(out));

	if (speedA > settings.perimeterTrackerSpeed) {
		speedA = settings.perimeterTrackerSpeed;
	}                // limit upper and lower speed
	if (speedB > settings.perimeterTrackerSpeed)
		speedB = settings.perimeterTrackerSpeed;

	if (speedA < 1000)
		speedA = 1000;
	if (speedB < 1000)
		speedB = 1000;

	if (BWF2_Status == OUTSIDE) {
		if (BWF1_Status == OUTSIDE) {
			MOTOR_LEFT_BACKWARD = settings.perimeterTrackerSpeed * 0.90;            // if both boundary sensors are OUTSIDE, reverse M1 motor, this logic needs to be changed if docking is to the right
			MOTOR_LEFT_FORWARD = 0;
			HAL_Delay(200);
		} else if (BWF1_Status == INSIDE) {
			MOTOR_LEFT_BACKWARD = 0;
			MOTOR_LEFT_FORWARD = speedB;
		}

		MOTOR_RIGHT_FORWARD = speedA;
		MOTOR_RIGHT_BACKWARD = 0;
	}

	if (BWF2_Status == INSIDE) {
		MOTOR_LEFT_BACKWARD = 0;
		MOTOR_LEFT_FORWARD = speedA;

		MOTOR_RIGHT_FORWARD = speedB;
		MOTOR_RIGHT_BACKWARD = 0;
	}
}

void parseCommand_Console(void) {
	// Parse commands from the Console

	char Command[64] = {"\0"};

	for (uint8_t x = 0; x < sizeof(ConsoleBuffer); x++) {
		if (ConsoleBuffer[x] == 13) {
			if (ConsoleBuffer[0] == 13) {
				memcpy(Command, "DISABLE", 7);
			} else {
				memcpy(Command, ConsoleBuffer, x);
			}
			sprintf(msg, "%s\r\n", Command);
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
				sprintf(msg, "%.2f\r\n", Voltage);
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
					sprintf(msg, "%f\r\n", validSignature[x]);
					Serial_Console(msg);
				}
			}
			if (strcmp(Command, "SHOW GUIDE") == 0) {
				Serial_Console("Guide will be printed in 8 seconds, please start your plotter before.\r\n");
				HAL_Delay(8000);
				for (int x = 0; x < SIGNATURE_LEN; x++) {
					sprintf(msg, "%f\r\n", validGuide[x]);
					Serial_Console(msg);
				}
			}
			if (strcmp(Command, "EXPORT SIG") == 0) {
				Serial_Console("Signature exported as an array\r\n");
				Serial_Console("{ ");
				for (int x = 0; x < SIGNATURE_LEN; x++) {
					sprintf(msg, "%f,", validSignature[x]);
					Serial_Console(msg);
				}
				Serial_Console(" };\r\n");
			}
			if (strcmp(Command, "EXPORT GUIDE") == 0) {
				Serial_Console("Guide exported as an array\r\n");
				Serial_Console("{ ");
				for (int x = 0; x < SIGNATURE_LEN; x++) {
					sprintf(msg, "%f,", validGuide[x]);
					Serial_Console(msg);
				}
				Serial_Console(" };\r\n");
			}
			if (strcmp(Command, "RECORD SIG") == 0) {
				Signature_Record = TRUE;
				Serial_Console("Done!\r\n");
			}
			if (strcmp(Command, "RECORD GUIDE") == 0) {
				Guide_Record = TRUE;
				Serial_Console("Done!\r\n");
			}
			if (strcmp(Command, "TEST LEFT MOTOR") == 0) {
				MOTOR_LEFT_BACKWARD = 0;
				MOTOR_LEFT_FORWARD = 2600;
				HAL_Delay(3000);
				MotorStop();
				Serial_Console("Done.\r\n");
			}
			if (strcmp(Command, "TEST RIGHT MOTOR") == 0) {
				MOTOR_RIGHT_FORWARD = 2600;
				MOTOR_RIGHT_BACKWARD = 0;
				HAL_Delay(3000);
				MotorStop();
				Serial_Console("Done.\r\n");
			}
            if (strcmp(Command, "DOCK RIGHT") == 0) {
                DockRightSeq();
            }
            if (strncmp(Command, "MOTOR RIGHT", 11) == 0) {
                int ms;
                char cmd1[5], cmd2[5];
                sscanf(Command, "%s %s %d ", cmd1, cmd2, &ms);

                if (ms < 7000)
                MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, ms);
            }
            if (strncmp(Command, "MOTOR LEFT", 10) == 0) {
                int ms;
                char cmd1[5], cmd2[4];
                sscanf(Command, "%s %s %d ", cmd1, cmd2, &ms);
                if (ms < 7000)
                MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, ms);
            }
            if (strncmp(Command, "MOTOR BACKWARDS", 15) == 0) {
                int ms;
                char cmd1[5], cmd2[9];
                sscanf(Command, "%s %s %d ", cmd1, cmd2, &ms);
                if (ms < 10000)
                MotorBackwardImpl(settings.motorMinSpeed, settings.motorMaxSpeed, ms, true);
            }
            if (strncmp(Command, "MOTOR FORWARD", 13) == 0) {
                int ms;
                char cmd1[5], cmd2[7];
                sscanf(Command, "%s %s %d ", cmd1, cmd2, &ms);
                MotorForwardImpl(settings.motorMinSpeed, settings.motorMaxSpeed, true);
                HAL_Delay(ms);
                MotorStop();
            }
            if (strcmp(Command, "MOTOR STOP") == 0) {
                MotorStop();
            }

			if (strcmp(Command, "REBOOT") == 0) {
				Serial_Console("Rebooting...\r\n");
				HAL_Delay(500);
				NVIC_SystemReset();
			}
            if (strcmp(Command, "UNDOCK") == 0) {
                undock(true);
            }
            if (strcmp(Command, "UNDOCK AND STOP") == 0) {
                lastGuideTrackingLapse = 0;
                undock(true);
                MasterSwitch = 0;
                MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, 1500);
                MotorForwardImpl(settings.motorMinSpeed, settings.motorMaxSpeed, true);
                HAL_Delay(3000);
                MotorStop();
            }
			if (strcmp(Command, "LOCK DOCKING") == 0) {
				Serial_Console("Docking LOCKED.\r\n");
				Docked_Locked = 1;
			}
			if (strcmp(Command, "UNLOCK DOCKING") == 0) {
				Serial_Console("Docking UNLOCKED.\r\n");
				Docked_Locked = 0;
			}
			if (strcmp(Command, "LOAD CONFIG") == 0) {
				settings = read_all_settings();
				Serial_Console("Config loaded.\r\n");
			}
			if (strcmp(Command, "SHOW CONFIG") == 0) {
				show_config(settings);
			}
			if (strcmp(Command, "SHOW ERRORS") == 0) {
				show_error();
			}
			if (strcmp(Command, "ADD ERROR") == 0) {
				add_error_event("This is a test error!");
			}

			if (strcmp(Command, "CLEAR ERRORS") == 0) {
				clear_errors();
			}
			if (strcmp(Command, "SAVE CONFIG") == 0) {
				write_all_settings(settings);
				settings = read_all_settings();
				Serial_Console("Settings saved.\r\n");
			}
			if (strcmp(Command, "SAVE DEFAULT CONFIG") == 0) {
				save_default_settings(board_revision);
				settings = read_all_settings();
				Serial_Console("Default settings saved.\r\n");
			}
			if (strcmp(Command, "SHOW CURRENT") == 0) {
				sprintf(msg, "M1: %.2f\r\nM2: %.2f\r\nC1: %.2f\r\n", M1_amp, M2_amp, C1_amp);
				Serial_Console(msg);
			}
			if (strcmp(Command, "STOP MOTORS") == 0) {
				MotorStop();
			}
			if (strcmp(Command, "RUN MOTORS FORWARD") == 0) {
				State = FORWARD;
				MOTOR_LEFT_BACKWARD = 0;
				MOTOR_LEFT_FORWARD = settings.motorMinSpeed;
				MOTOR_RIGHT_FORWARD = settings.motorMinSpeed;
				MOTOR_RIGHT_BACKWARD = 0;
			}
			if (strcmp(Command, "RUN MOTORS REVERSE") == 0) {
				State = BACKWARD;
				MOTOR_LEFT_BACKWARD = settings.motorMinSpeed;
				MOTOR_LEFT_FORWARD = 0;
				MOTOR_RIGHT_FORWARD = 0;
				MOTOR_RIGHT_BACKWARD = settings.motorMinSpeed;
			}
			if (strncmp(Command, "SET PITCH COMP", 14) == 0) {
				float pitch;
				char cmd1[3], cmd2[5], cmd3[4];
				sscanf(Command, "%s %s %s %f", cmd1, cmd2, cmd3, &pitch);
				settings.pitch_comp = pitch;
			}
			if (strncmp(Command, "SET ROLL COMP", 13) == 0) {
				float roll;
				char cmd1[3], cmd2[4], cmd3[4];
				sscanf(Command, "%s %s %s %f", cmd1, cmd2, cmd3, &roll);
				settings.roll_comp = roll;
			}
			if (strncmp(Command, "SET STEERING CORRECTION", 23) == 0) {
				uint16_t correction;
				char cmd1[3], cmd2[8], cmd3[10];
				sscanf(Command, "%s %s %s %hd", cmd1, cmd2, cmd3, &correction);
				settings.steering_correction = correction;
			}
			if (strncmp(Command, "SET PROXIMITY SPEED", 19) == 0) {
				float speed;
				char cmd1[3], cmd2[9], cmd3[5];
				sscanf(Command, "%s %s %s %f", cmd1, cmd2, cmd3, &speed);
				settings.proximitySpeed = speed;
			}
			if (strncmp(Command, "SET VOLTAGE MULTIPLY", 20) == 0) {
				float multiply;
				char cmd1[3], cmd2[7], cmd3[8];
				sscanf(Command, "%s %s %s %f", cmd1, cmd2, cmd3, &multiply);
				settings.voltageMultiply = multiply;
			}
			if (strncmp(Command, "SET MOTOR MAX LIMIT", 19) == 0) {
				float limit;
				char cmd1[3], cmd2[5], cmd3[3], cmd4[5];
				sscanf(Command, "%s %s %s %s %f", cmd1, cmd2, cmd3, cmd4, &limit);
				settings.Motor_Max_Limit = limit;
			}
			if (strncmp(Command, "SET MOTOR MIN LIMIT", 19) == 0) {
				float limit;
				char cmd1[3], cmd2[5], cmd3[3], cmd4[5];
				sscanf(Command, "%s %s %s %s %f", cmd1, cmd2, cmd3, cmd4, &limit);
				settings.Motor_Min_Limit = limit;
			}
			if (strncmp(Command, "SET MOTOR MAX SPEED", 19) == 0) {
				int speed;
				char cmd1[3], cmd2[5], cmd3[3], cmd4[5];
				sscanf(Command, "%s %s %s %s %d", cmd1, cmd2, cmd3, cmd4, &speed);
				settings.motorMaxSpeed = speed;
			}
			if (strncmp(Command, "SET MOTOR MIN SPEED", 19) == 0) {
				int speed;
				char cmd1[3], cmd2[5], cmd3[3], cmd4[5];
				sscanf(Command, "%s %s %s %s %d", cmd1, cmd2, cmd3, cmd4, &speed);
				settings.motorMinSpeed = speed;
			}
			if (strncmp(Command, "SET ROLL TILT COMP", 18) == 0) {
				int comp;
				char cmd1[3], cmd2[4], cmd3[4], cmd4[4];
				sscanf(Command, "%s %s %s %s %d", cmd1, cmd2, cmd3, cmd4, &comp);
				settings.roll_tilt_comp = comp;
			}
			if (strncmp(Command, "SET PERIMETER SPEED", 19) == 0) {
				int speed;
				char cmd1[3], cmd2[9], cmd3[5];
				sscanf(Command, "%s %s %s %d", cmd1, cmd2, cmd3, &speed);
				settings.perimeterTrackerSpeed = speed;
			}
			if (strncmp(Command, "SET PERIMETER CUT RATIO", 23) == 0) {
				int ratio;
				char cmd1[3], cmd2[9], cmd3[3], cmd4[5];
				sscanf(Command, "%s %s %s %s %d", cmd1, cmd2, cmd3, cmd4, &ratio);
				settings.cut_perimeter_ratio = ratio;
			}
			if (strncmp(Command, "SET ADC LEVEL", 13) == 0) {
				int adc;
				char cmd1[3], cmd2[3], cmd3[5];
				sscanf(Command, "%s %s %s %d", cmd1, cmd2, cmd3, &adc);
				settings.adcLevel = adc;
			}
			if (strncmp(Command, "SET CUTTER SPEED", 16) == 0) {
				int speed;
				char cmd1[3], cmd2[5], cmd3[3];
				sscanf(Command, "%s %s %s %d", cmd1, cmd2, cmd3, &speed);
				settings.cutterSpeed = speed;
			}
			if (strncmp(Command, "SET BOUNDARY TIMEOUT", 20) == 0) {
				int limit;
				char cmd1[3], cmd2[8], cmd3[7];
				sscanf(Command, "%s %s %s %d ", cmd1, cmd2, cmd3, &limit);
				settings.Boundary_Timeout = limit;
			}
			if (strncmp(Command, "SET OVERTURN LIMIT", 18) == 0) {
				int limit;
				char cmd1[3], cmd2[8], cmd3[5];
				sscanf(Command, "%s %s %s %d ", cmd1, cmd2, cmd3, &limit);
				settings.Overturn_Limit = limit;
			}
			if (strncmp(Command, "SET OUTSIDE LIMIT", 17) == 0) {
				int limit;
				char cmd1[3], cmd2[7], cmd3[5];
				sscanf(Command, "%s %s %s %d ", cmd1, cmd2, cmd3, &limit);
				settings.Outside_Threshold = limit;
			}
			if (strncmp(Command, "SET BAT HIGH", 12) == 0) {
				float limit;
				char cmd1[3], cmd2[3], cmd3[4];
				sscanf(Command, "%s %s %s %f ", cmd1, cmd2, cmd3, &limit);
				settings.Battery_High_Limit = limit;
			}
			if (strncmp(Command, "SET BAT LOW", 11) == 0) {
				float limit;
				char cmd1[3], cmd2[3], cmd3[3];
				sscanf(Command, "%s %s %s %f ", cmd1, cmd2, cmd3, &limit);
				settings.Battery_Low_Limit = limit;
			}
			if (strncmp(Command, "SET BAT CHARGER TIME", 20) == 0) {
				int minutes;
				char cmd1[3], cmd2[3], cmd3[7], cmd4[4];
				sscanf(Command, "%s %s %s %s %d ", cmd1, cmd2, cmd3, cmd4, &minutes);
				settings.BatteryChargeTime = minutes;
			}
			if (strncmp(Command, "SET BWF OUT", 11) == 0) {
				float limit;
				char cmd1[3], cmd2[3], cmd3[3];
				sscanf(Command, "%s %s %s %f ", cmd1, cmd2, cmd3, &limit);
				settings.Signal_Integrity_OUT = limit;
			}
			if (strncmp(Command, "SET BWF IN", 10) == 0) {
				float limit;
				char cmd1[3], cmd2[3], cmd3[2];
				sscanf(Command, "%s %s %s %f ", cmd1, cmd2, cmd3, &limit);
				settings.Signal_Integrity_IN = limit;
			}
            if (strncmp(Command, "SET BWF FLIP", 12) == 0) {
                uint8_t flip;
                char cmd1[3], cmd2[3], cmd3[4];
                sscanf(Command, "%s %s %s %hhd", cmd1, cmd2, cmd3, &flip);
                settings.bwf_flip = flip;
                calculate_bwf_flip(&settings);
            }
			if (strncmp(Command, "SET GUIDE IN", 12) == 0) {
				float limit;
				char cmd1[3], cmd2[5], cmd3[2];
				sscanf(Command, "%s %s %s %f ", cmd1, cmd2, cmd3, &limit);
				settings.Guide_Integrity_IN = limit;
			}
			if (strncmp(Command, "SET GUIDE OUT", 13) == 0) {
				float limit;
				char cmd1[3], cmd2[5], cmd3[3];
				sscanf(Command, "%s %s %s %f ", cmd1, cmd2, cmd3, &limit);
				settings.Guide_Integrity_OUT = limit;
			}
			if (strncmp(Command, "SET CHARGE DETECTION", 20) == 0) {
				int limit;
				char cmd1[3], cmd2[6], cmd3[9];
				sscanf(Command, "%s %s %s %d", cmd1, cmd2, cmd3, &limit);
				settings.HoldChargeDetection = limit;
			}
			if (strncmp(Command, "SET CUTTER LIMIT", 16) == 0) {
				float limit;
				char cmd1[3], cmd2[6], cmd3[5];
				sscanf(Command, "%s %s %s %f ", cmd1, cmd2, cmd3, &limit);
				settings.Cutter_Limit = limit;
			}
			if (strncmp(Command, "SET MOTOR TURN STATIC", 21) == 0) {
				uint16_t time;
				char cmd1[3], cmd2[5], cmd3[4], cmd4[6];
				sscanf(Command, "%s %s %s %s %hd", cmd1, cmd2, cmd3, cmd4, &time);
				settings.motorTurnStatic_time =  time;
			}
			if (strncmp(Command, "SET MOTOR TURN RANDOM", 21) == 0) {
				uint16_t time;
				char cmd1[3], cmd2[5], cmd3[4], cmd4[6];
				sscanf(Command, "%s %s %s %s %hd", cmd1, cmd2, cmd3, cmd4, &time);
				settings.motorTurnRandom_time =  time;
			}
            if (strncmp(Command, "SET MOTOR BACKWARD", 18) == 0) {
                uint16_t time;
                char cmd1[3], cmd2[5], cmd3[8];
                sscanf(Command, "%s %s %s %hd", cmd1, cmd2, cmd3, &time);
                settings.motorBackward_time =  time;
            }
            if (strncmp(Command, "SET LAB1", 8) == 0) {
                uint16_t lab;
                char cmd1[3], cmd2[4];
                sscanf(Command, "%s %s %hd", cmd1, cmd2, &lab);
                settings.lab_1 =  lab;
            }
            if (strncmp(Command, "SET LAB2", 8) == 0) {
                uint16_t lab;
                char cmd1[3], cmd2[4];
                sscanf(Command, "%s %s %hd", cmd1, cmd2, &lab);
                settings.lab_2 =  lab;
            }
            if (strncmp(Command, "SET LAB3", 8) == 0) {
                uint16_t lab;
                char cmd1[3], cmd2[4];
                sscanf(Command, "%s %s %hd", cmd1, cmd2, &lab);
                settings.lab_3 =  lab;
            }
			if (strncmp(Command, "SET MOTOR LIMIT", 15) == 0) {
				float limit;
				char cmd1[3], cmd2[5], cmd3[5];
				sscanf(Command, "%s %s %s %f ", cmd1, cmd2, cmd3, &limit);
				settings.Motor_Limit = limit;
			}
			if (strncmp(Command, "SET MOVEMENT LIMIT", 18) == 0) {
				float limit;
				char cmd1[3], cmd2[8], cmd3[5];
				sscanf(Command, "%s %s %s %f ", cmd1, cmd2, cmd3, &limit);
				settings.movement = limit;
			}
			if (strncmp(Command, "SET MOVEMENT COUNT LIMIT", 24) == 0) {
				int limit;
				char cmd1[3], cmd2[8], cmd3[5], cmd4[5];
				sscanf(Command, "%s %s %s %s %d ", cmd1, cmd2, cmd3, cmd4, &limit);
				settings.move_count_limit = limit;
			}
			if (strncmp(Command, "SET BUMPER COUNT LIMIT", 22) == 0) {
				int limit;
				char cmd1[3], cmd2[6], cmd3[5], cmd4[5];
				sscanf(Command, "%s %s %s %s %d ", cmd1, cmd2, cmd3, cmd4, &limit);
				settings.bumper_count_limit = limit;
			}
			if (strncmp(Command, "SET UNDOCK BACKING SECONDS", 26) == 0) {
				int seconds;
				char cmd1[3], cmd2[6], cmd3[5], cmd4[5];
				sscanf(Command, "%s %s %s %s %d ", cmd1, cmd2, cmd3, cmd4, &seconds);
				settings.undock_backing_seconds = seconds;
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
				settings.kp = pid_kp;
			}
			if (strncmp(Command, "SET KI", 6) == 0) {
				float pid_ki;
				char cmd1[3], cmd2[2];
				sscanf(Command, "%s %s %f", cmd1, cmd2, &pid_ki);
				settings.ki = pid_ki;
			}
			if (strncmp(Command, "SET KD", 6) == 0) {
				float pid_kd;
				char cmd1[3], cmd2[2];
				sscanf(Command, "%s %s %f", cmd1, cmd2, &pid_kd);
				settings.kd = pid_kd;
			}
			if (strncmp(Command, "SET WORKING START", 17) == 0) {
				int start;
				char cmd1[3], cmd2[7], cmd3[5];
				sscanf(Command, "%s %s %s %d", cmd1, cmd2, cmd3, &start);
				settings.WorkingHourStart = start;
			}
			if (strncmp(Command, "SET WORKING END", 15) == 0) {
				int end;
				char cmd1[3], cmd2[7], cmd3[3];
				sscanf(Command, "%s %s %s %d", cmd1, cmd2, cmd3, &end);
				settings.WorkingHourEnd = end;
			}
			if (strncmp(Command, "SET MAG VALUE", 13) == 0) {
				int magValue;
				char cmd1[3], cmd2[3], cmd3[5];
				sscanf(Command, "%s %s %s %d", cmd1, cmd2, cmd3, &magValue);
				settings.magValue = magValue;
			}
			if (strncmp(Command, "SET MAGMIN VALUE", 16) == 0) {
				int magMinValue;
				char cmd1[3], cmd2[6], cmd3[5];
				sscanf(Command, "%s %s %s %d", cmd1, cmd2, cmd3, &magMinValue);
				settings.magMinValue = magMinValue;
			}
			if (strncmp(Command, "SET HIGHGRASS LIMIT", 19) == 0) {
				float highgrass;
				char cmd1[3], cmd2[9], cmd3[5];
				sscanf(Command, "%s %s %s %f", cmd1, cmd2, cmd3, &highgrass);
				settings.highgrass_Limit = highgrass;
			}
			if (strncmp(Command, "SET USE GUIDE WIRE", 18) == 0) {
				uint8_t guide_wire;
				char cmd1[3], cmd2[3], cmd3[5], cmd4[4];
				sscanf(Command, "%s %s %s %s %hhd", cmd1, cmd2, cmd3, cmd4, &guide_wire);
				settings.use_guide_wire = guide_wire;
			}
			if (strncmp(Command, "SET MULTI MODE", 14) == 0) {
				uint8_t multi_mode;
				char cmd1[3], cmd2[5], cmd3[4];
				sscanf(Command, "%s %s %s %hhd", cmd1, cmd2, cmd3, &multi_mode);
				settings.multi_mode = multi_mode;
			}
			if (strncmp(Command, "SET NEXT GUIDE TRACK", 20) == 0) {
				uint16_t time;
				char cmd1[3], cmd2[5], cmd3[3], cmd4[5];
				sscanf(Command, "%s %s %s %s %hd", cmd1, cmd2, cmd3, cmd4, &time);
				lastGuideTrackingLapse = time * 1000;
			}
			if (strncmp(Command, "SHOW NEXT GUIDE TRACK", 21) == 0) {
				sprintf(msg, "Next guide tracking time: %lu s\r\n", lastGuideTrackingLapse/1000);
				Serial_Console(msg);
			}
            if (strncmp(Command, "SET MOTOR BACKWARD", 18) == 0) {
                uint16_t time;
                char cmd1[3], cmd2[5], cmd3[8];
                sscanf(Command, "%s %s %s %hd", cmd1, cmd2, cmd3, &time);
                settings.motorBackward_time =  time;
            }
			if (strcmp(Command, "DISABLE") == 0) {
				MasterSwitch = 0;
				add_error_event("User typed disable");
				Serial_Console("RazorBoard DISABLED.\r\n");
				Serial_Console("Please type <help> to see available commands\r\n");
			}

			if (strcmp(Command, "ENABLE") == 0) {
				MasterSwitch = 1;
				add_error_event("User typed enable");
				Serial_Console("RazorBoard ENABLED. STEP AWAY FROM THE VEHICLE!\r\n");
				Initial_Start = 0;
			}
			if (strcmp(Command, "TRACK PERIMETER") == 0) {
				perimeterTracking = 1;
				Serial_Console("Perimeter tracking ENABLED\r\n");
			}
            if (strcmp(Command, "TRACK GUIDE") == 0) {
                searchingForGuide = 1;
                guideTrackingOut = 0;
                Serial_Console("Guide tracking ENABLED\r\n");
            }
            if (strcmp(Command, "TRACK GUIDE NOW") == 0) {
                searchingForGuide = 0;
                guideTrackingOut = 0;
                guideTrackingActive = 1;
                guideTrackingStartedAt = 0;
                Serial_Console("Guide tracking now ENABLED\r\n");
            }
            if (strcmp(Command, "CUTTER OFF") == 0) {
                cutterOFF();
            }
            if (strcmp(Command, "CUTTER ON") == 0) {
                cutterON();
            }
            if (strcmp(Command, "TRACK GUIDE OUT") == 0) {
                searchingForGuide = 1;
                guideTrackingOut = 1;
                Serial_Console("Guide tracking out ENABLED\r\n");
            }
            if (strcmp(Command, "STOP TRACK GUIDE") == 0) {
                searchingForGuide = 0;
                guideTrackingActive = 0;
                guideTrackingOut = 0;
                Serial_Console("Guide tracking DISABLED\r\n");
                MotorStop();
            }
			if (strcmp(Command, "RESETI2C") == 0) {
				reInitIMU();
			}
			if (strcmp(Command, "SCAN I2C") == 0) {
				scanner(hi2c1);
				scanner(hi2c2);
			}
			if (strcmp(Command, "HELP") == 0) {
				help();
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
			memcpy(Command, PIBuffer, x);
			sprintf(msg, "%s\r\n", Command);
			if (strcmp(Command, "RUN") == 0) {
				MasterSwitch = 1;
				Serial_RPi("Status: RUN\r\n");
				Serial_Console("Status: RUN\r\n");
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

	HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), 100);
}

void Serial_RPi(char *msg) {
	// Write to Raspberry PI

	HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), 100);
}

uint8_t CheckSecurity(void) {
	// Check security, what is our status with the boundary signals

	CheckBWF();
    CheckBWF_Rear();

	if (State == BACKWARD) {
		if (BWF3_Status == OUTSIDE) {
			return SECURITY_BACKWARD_OUTSIDE;
		}
	}

	if (fabs(mpu.pitch) >= settings.Overturn_Limit || fabs(mpu.roll) >= settings.Overturn_Limit) {
        if (!isOverturned) {
            isOverturned = true;
            sprintf(emsg, "Overturn: pitch %.1f roll %.1f", mpu.pitch, mpu.roll);
            add_error_event(emsg);
            MotorHardBrake();
            cutterHardBreak();
            HAL_Delay(100);
            getIMUOrientation();

            CheckBWF();
            CheckBWF_Rear();

            // After overturn
            if (!(fabs(mpu.pitch) >= settings.Overturn_Limit || fabs(mpu.roll) >= settings.Overturn_Limit)) {
                if (BWF1_Status == INSIDE && BWF2_Status == INSIDE && BWF3_Status == INSIDE) {
                    MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, 2000);
                }

                CheckBWF();
                CheckBWF_Rear();

                if (BWF1_Status == INSIDE && BWF2_Status == INSIDE && BWF3_Status == INSIDE) {
                    MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, 800);
                }
            }
        } else {
            MOTOR_LEFT_BACKWARD = 0;
            MOTOR_LEFT_FORWARD = 0;
            MOTOR_RIGHT_FORWARD = 0;
            MOTOR_RIGHT_BACKWARD = 0;
            TIM3->CCR1 = 0;
            TIM3->CCR2 = 0;
        }

        return SECURITY_IMU_FAIL;
	} else {
        isOverturned = false;
	}

    if (BWF1_Status == INSIDE && BWF2_Status == INSIDE) {
        Fail_Timer = 0;
    } else if (Fail_Timer == 0) {
        Fail_Timer = HAL_GetTick();
    }

    if (Collision == 1 && IsCollisionChecking == 0) {
        return SECURITY_BUMPER;
    }

	if (HAL_GetTick() - Boundary_Timer >= (settings.Boundary_Timeout * 1000)) {
		BWF1_Status = NOSIGNAL;
		BWF2_Status = NOSIGNAL;
		State = FAIL;
		Security = NOSIGNAL;
		return SECURITY_NOSIGNAL;
	}

	if (Initial_Start == 0) {
		move_timer = HAL_GetTick();
	}

	if ((MOTOR_LEFT_FORWARD >= (settings.motorMaxSpeed * 0.5) || MOTOR_RIGHT_FORWARD >= (settings.motorMaxSpeed * 0.5)) && mpu.movement < settings.movement) {
		if (HAL_GetTick() - move_timer >= 5000) {
			move_count++;
			return SECURITY_MOVEMENT;
		}
	} else {
		move_timer = HAL_GetTick();
	}
	if (BWF1_Status == INSIDE || BWF2_Status == INSIDE) {
		OUTSIDE_timer = HAL_GetTick();        // We are inside with at least one sensor, reset OUTSIDE_timer
	}

	if (BWF1_Status == INSIDE && BWF2_Status == INSIDE) {
		Security = INSIDE;
		return SECURITY_OK;
	} else if ( BWF1_Status == OUTSIDE || BWF2_Status == OUTSIDE ) {
			Security = OUTSIDE;
			return SECURITY_FAIL;
		}

	return SECURITY_FAIL;
}

void cutterHardBreak() {
	// Cutter disc hard brake

	TIM3->CCR1 = 3359;        // Motor will hard brake when both "pins" go HIGH
	TIM3->CCR2 = 3359;
	delay(500);
	cutterOFF();
}

void cutterON(void) {
    if (Voltage <= settings.Battery_Low_Limit)
    {
        add_error_event("Cutter stopped - low bat");
        cutterOFF();
        return;
    }

    if (fabs(mpu.pitch) >= settings.Overturn_Limit || fabs(mpu.roll) >= settings.Overturn_Limit) {
        sprintf(emsg, "Cut on stopped: pitch %.1f roll %.1f", mpu.pitch, mpu.roll);
        add_error_event(emsg);
        cutterOFF();
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
			TIM3->CCR1 = cutterSpeed;
			TIM3->CCR2 = 0;
		} else {
			TIM3->CCR1 = 0;
			TIM3->CCR2 = cutterSpeed;
		}
		delay(2);
	}
}

void cutterOFF(void) {
	cutterStatus = 0;

	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
}

void CheckBWF_Rear() {
	float BWF3_Mixed_Signal = 0;
	float BWF3_Received_Signal = 0;
	uint16_t myID = 0;
	uint8_t BWF3_reply = 0;
	float Match_Signal = 0;
	float Result_Signal = 0;
	float BWF3_Verdict_Signal = 0.0;
	int count = 0;

	for (int x = 0; x < ADC_SAMPLE_LEN; x++) {
		if (x % 2) {
			count++;
		} else {
			BWF3[count] = ADC_REAR_BUFFER[x] - settings.adcLevel;        // Normalize the ADC signal
		}
	}

	FIR_REAR();

	for (uint16_t idx = 0; idx < 128; idx++) {
		if (BWF3_reply == 1) {
			break;
		}
		myID = 0;
		BWF3_Mixed_Signal = 0;
		BWF3_Received_Signal = 0;
		Match_Signal = 0;


		for (int x = idx; x < (idx + SIGNATURE_LEN - 1); x++) {
			BWF3_Mixed_Signal += (BWF3[x] * validSignature[myID]);
			BWF3_Received_Signal += BWF3[x] * BWF3[x];

			Match_Signal += validSignature[myID] * validSignature[myID];
			myID++;
		}

		arm_sqrt_f32((BWF3_Received_Signal * Match_Signal), &Result_Signal);
		BWF3_Verdict_Signal = (BWF3_Mixed_Signal / Result_Signal) * settings.bwf3_flip;

		if (BWF3_Verdict_Signal >= settings.Signal_Integrity_IN && BWF3_reply == 0) {
			BWF3_Status = INSIDE;
			Boundary_Timer = HAL_GetTick();
			BWF3_reply = 1;
			bwf3_inside++;
		} else if (BWF3_Verdict_Signal <= settings.Signal_Integrity_OUT && BWF3_reply == 0) {
			BWF3_Status = OUTSIDE;
			Boundary_Timer = HAL_GetTick();
			BWF3_reply = 1;
			bwf3_outside++;
		}
	}

    BWF3_Status = BWF3_Status;
    CheckBWF_RearGuide();
}

void CheckBWF_RearGuide() {
    float BWF3_Mixed_Signal = 0;
    float BWF3_Received_Signal = 0;
    uint16_t myID = 0;
    uint8_t BWF3_reply = 0;
    float Match_Signal = 0;
    float Result_Signal = 0;
    float BWF3_Verdict_Signal = 0.0;
    int count = 0;

    for (int x = 0; x < ADC_SAMPLE_LEN; x++) {
        if (x % 2) {
            count++;
        } else {
            BWF3[count] = ADC_REAR_BUFFER[x] - settings.adcLevel;        // Normalize the ADC signal
        }
    }

    FIR_REAR();

    for (uint16_t idx = 0; idx < 128; idx++) {
        if (BWF3_reply == 1) {
            break;
        }
        myID = 0;
        BWF3_Mixed_Signal = 0;
        BWF3_Received_Signal = 0;
        Match_Signal = 0;

        for (int x = idx; x < (idx + SIGNATURE_LEN - 1); x++) {
            BWF3_Mixed_Signal += (BWF3[x] * validGuide[myID]);
            BWF3_Received_Signal += BWF3[x] * BWF3[x];

            Match_Signal += validGuide[myID] * validGuide[myID];
            myID++;
        }

        arm_sqrt_f32((BWF3_Received_Signal * Match_Signal), &Result_Signal);
        BWF3_Verdict_Signal = (BWF3_Mixed_Signal / Result_Signal) * settings.bwf3_flip;

        if (BWF3_Verdict_Signal >= settings.Guide_Integrity_IN && BWF3_reply == 0) {
            BWF3_guide_status = INSIDE;
            Boundary_Timer = HAL_GetTick();
            BWF3_reply = 1;
            bwf3guide_inside++;
        } else if (BWF3_Verdict_Signal <= settings.Guide_Integrity_OUT && BWF3_reply == 0) {
            BWF3_guide_status = OUTSIDE;
            Boundary_Timer = HAL_GetTick();
            BWF3_reply = 1;
            bwf3guide_outside++;
        }
    }

    BWF3_guide_status = BWF3_guide_status;
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

	int BWF_duration;
	BWF_duration = HAL_GetTick();

	float BWF1_Mixed_Signal = 0;
	float BWF1_Received_Signal = 0;
	float BWF2_Mixed_Signal = 0;
	float BWF2_Received_Signal = 0;

	float Guide_BWF1_Mixed_Signal = 0;
	float Guide_BWF2_Mixed_Signal = 0;

	uint16_t myID = 0;
	uint8_t BWF1_reply = 0;
	uint8_t BWF2_reply = 0;

	uint8_t Guide_BWF1_reply = 0;
	uint8_t Guide_BWF2_reply = 0;

	float Match_Signal = 0;
	float Result_Signal = 0;
	float BWF1_Verdict_Signal = 0.0;
	float BWF2_Verdict_Signal = 0.0;

	float Guide_Match_Signal = 0;
	float Guide_Result_Signal = 0;
	float Guide_BWF1_Verdict_Signal = 0.0;
	float Guide_BWF2_Verdict_Signal = 0.0;

	int count = 0;

	for (int x = 0; x < ADC_SAMPLE_LEN; x++) {
		if (x % 2) {
			BWF2[count] = ADC_BUFFER[x] - settings.adcLevel;        // Normalize the ADC signal
			count++;
		} else {
			BWF1[count] = ADC_BUFFER[x] - settings.adcLevel;        // Normalize the ADC signal
		}
	}

	FIR_LEFT();         // Run FIR on left BWF	(BWF1)
	FIR_RIGHT();    	// Run FIR on right BWF	(BWF2)

	if (Signature_Record == TRUE) {
		for (uint16_t x = 0; x < SIGNATURE_LEN; x++) {
			validSignature[x] = BWF1[x];
		}
		Signature_Record = FALSE;
	}

	if (Guide_Record == TRUE) {
		for (uint16_t x = 0; x < SIGNATURE_LEN; x++) {
			validGuide[x] = BWF1[x];
		}
		Guide_Record = FALSE;
	}

	for (uint16_t idx = 0; idx < 127; idx++) {
		if ( (BWF1_reply == 1 && BWF2_reply == 1) || (Guide_BWF1_reply == 1 && Guide_BWF2_reply == 1)) {
			break;
		}
		myID = 0;
		BWF1_Mixed_Signal = 0;
		BWF1_Received_Signal = 0;
		BWF2_Mixed_Signal = 0;
		BWF2_Received_Signal = 0;
		Match_Signal = 0;

		Guide_BWF1_Mixed_Signal = 0;
		Guide_BWF2_Mixed_Signal = 0;
		Guide_Match_Signal = 0;


		for (int x = idx; x < (idx + SIGNATURE_LEN - 1); x++) {

				BWF1_Mixed_Signal += (BWF1[x] * validSignature[myID]);
				BWF2_Mixed_Signal += (BWF2[x] * validSignature[myID]);
				Match_Signal += validSignature[myID] * validSignature[myID];

				Guide_BWF1_Mixed_Signal += (BWF1[x] * validGuide[myID]);
				Guide_BWF2_Mixed_Signal += (BWF2[x] * validGuide[myID]);
				Guide_Match_Signal += validGuide[myID] * validGuide[myID];

			BWF1_Received_Signal += BWF1[x] * BWF1[x];
			BWF2_Received_Signal += BWF2[x] * BWF2[x];

			myID++;
		}

			arm_sqrt_f32((BWF1_Received_Signal * Match_Signal), &Result_Signal);
			BWF1_Verdict_Signal = (BWF1_Mixed_Signal / Result_Signal) * settings.bwf1_flip;
			arm_sqrt_f32((BWF2_Received_Signal * Match_Signal), &Result_Signal);
			BWF2_Verdict_Signal = (BWF2_Mixed_Signal / Result_Signal) * settings.bwf2_flip;

			arm_sqrt_f32((BWF1_Received_Signal * Guide_Match_Signal), &Guide_Result_Signal);
			Guide_BWF1_Verdict_Signal = (Guide_BWF1_Mixed_Signal / Guide_Result_Signal) * settings.bwf1_flip;
			arm_sqrt_f32((BWF2_Received_Signal * Guide_Match_Signal), &Guide_Result_Signal);
			Guide_BWF2_Verdict_Signal = (Guide_BWF2_Mixed_Signal / Guide_Result_Signal) * settings.bwf2_flip;

			if (Guide_BWF1_Verdict_Signal >= settings.Guide_Integrity_IN && Guide_BWF1_reply == 0) {
				BWF1_guide_status = OUTSIDE;
				Boundary_Timer = HAL_GetTick();
				bwf1guide_inside++;
				Guide_BWF1_reply = 1;
				CalcMagnitude(idx, 11);
			}
			else if (Guide_BWF1_Verdict_Signal <= settings.Guide_Integrity_OUT && Guide_BWF1_reply == 0) {
				BWF1_guide_status = INSIDE;
				Boundary_Timer = HAL_GetTick();
				bwf1guide_outside++;
				Guide_BWF1_reply = 1;
				CalcMagnitude(idx, 11);
			}

			if (Guide_BWF2_Verdict_Signal >= settings.Guide_Integrity_IN && Guide_BWF2_reply == 0) {
				BWF2_guide_status = OUTSIDE;
				Boundary_Timer = HAL_GetTick();
				bwf2guide_inside++;
				Guide_BWF2_reply = 1;
				CalcMagnitude(idx, 12);
			}
			else if (Guide_BWF2_Verdict_Signal <= settings.Guide_Integrity_OUT && Guide_BWF2_reply == 0) {
				BWF2_guide_status = INSIDE;
				Boundary_Timer = HAL_GetTick();
				bwf2guide_outside++;
				Guide_BWF2_reply = 1;
				CalcMagnitude(idx, 12);
			}

		if (BWF1_Verdict_Signal >= settings.Signal_Integrity_IN && BWF1_reply == 0) {
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
			CalcMagnitude(idx, 1);
		} else if (BWF1_Verdict_Signal <= settings.Signal_Integrity_OUT && BWF1_reply == 0) {
			BWF1_Status = OUTSIDE;
			Boundary_Timer = HAL_GetTick();
			BWF1_reply = 1;
			if (Initial_Start == 0) {
				Start_Threshold = 0;
			}
			bwf1_outside++;
			CalcMagnitude(idx, 1);
		}

		if (BWF2_Verdict_Signal >= settings.Signal_Integrity_IN && BWF2_reply == 0) {
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
			CalcMagnitude(idx, 2);
		} else if (BWF2_Verdict_Signal <= settings.Signal_Integrity_OUT && BWF2_reply == 0) {
			BWF2_Status = OUTSIDE;
			Boundary_Timer = HAL_GetTick();
			BWF2_reply = 1;
			if (Initial_Start == 0) {
				Start_Threshold = 0;
			}
			bwf2_outside++;
			CalcMagnitude(idx, 2);
		}
	}

//    BWF1_Status = BWF1_Status * settings.bwf1_flip;
//    BWF1_guide_status = BWF1_guide_status * settings.bwf1_flip;
//
//    BWF2_Status = BWF2_Status * settings.bwf2_flip;
//    BWF2_guide_status = BWF2_guide_status * settings.bwf2_flip;

	BWF_cycle = HAL_GetTick() - BWF_duration;
}
void ADC_Send(uint8_t Channel) {
	// Send ADC data

	unsigned char ADSwrite[3];

	memset(ADSwrite, 0, sizeof(ADSwrite));

	ADSwrite[0] = 0x01;
	ADSwrite[1] = Channel;
	ADSwrite[2] = 0x83;
    if (HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100) != HAL_OK) {
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
		return;
	}
	ADSwrite[0] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 1, 100) != HAL_OK) {
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
		return;
	}
}

int ADC_Receive() {
	// Receive ADC data

	unsigned char ADSwrite[2];
	int16_t reading;

	memset(ADSwrite, 0, sizeof(ADSwrite));

	if (HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 2, 100) != HAL_OK) {
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
		return 0;

	}

	reading = (ADSwrite[0] << 8 | ADSwrite[1]);

	if (reading < 0 || reading > 32767) {
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
		Speed = settings.motorCruiseSpeed;
		if (mag_near_bwf == 1) {
			Speed = round(Speed * settings.proximitySpeed);
		} else if (highgrass_slowdown == 1) {
			Speed = round(Speed * 0.8 - highgrass_level);
		}
	} else if (searchingForGuide == 1) {
		Speed = settings.motorCruiseSpeed * 0.95;
	} else {
        Speed = settings.motorCruiseSpeed;
    }

	// Target is on the Left side
	if (dir > 0) {
		int CorrectedSpeed = Speed - abs(diff);
		MOTOR_LEFT_FORWARD = CorrectedSpeed;
		MOTOR_RIGHT_FORWARD = Speed;
	}
	// Target is on the Right side
	else if (dir < 0) {
		int CorrectedSpeed = Speed - abs(diff);
		MOTOR_LEFT_FORWARD = Speed;
		MOTOR_RIGHT_FORWARD = CorrectedSpeed;
	}
	// Spot on! Full speed ahead Captain!
	else if (dir == 0) {
		MOTOR_LEFT_FORWARD = Speed;
		MOTOR_RIGHT_FORWARD = Speed;
	}
}



void MotorForward(uint16_t minSpeed, uint16_t maxSpeed) {
    MotorForwardImpl(minSpeed, maxSpeed, false);
}

void MotorForwardImpl(uint16_t minSpeed, uint16_t maxSpeed, bool force) {
	if (Docked == 1) return;
	State = FORWARD;

    if (CheckSecurity() == SECURITY_FAIL && !force) {
        MotorStop();
        return;
    }

    if (CheckSecurity() == SECURITY_BUMPER) {
        CheckChassi();
        return;
    }

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

        if (!force && CheckSecurity() == SECURITY_FAIL) {
            MotorStop();
            break;
        }

        if (CheckSecurity() == SECURITY_BUMPER) {
            CheckChassi();
            break;
        }
	}
}

void MotorBackward(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {
	MotorBackwardImpl(minSpeed, maxSpeed, time_ms, false);
}

void MotorBackwardImpl(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms, bool forced) {
	if (Docked == 1) return;
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
            add_error_event("Backed outside on acc");
			MotorHardBrake();
			delay(500);
			if (!isOverturned) {
                if (BWF1_Status == INSIDE && BWF2_Status == INSIDE) {
                    MotorForward(settings.motorMinSpeed, settings.motorMaxSpeed);
                } else if (BWF1_Status == INSIDE) {
                    MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time);
                } else if (BWF2_Status == INSIDE) {
                    MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time);
                }
			}
			delay(750);
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
            add_error_event("Backed outside on delay");
			MotorHardBrake();
			delay(500);
            if (BWF1_Status == INSIDE && BWF2_Status == INSIDE) {
                MotorForward(settings.motorMinSpeed, settings.motorMaxSpeed);
            } else if (BWF1_Status == INSIDE) {
                MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time);
            } else if (BWF2_Status == INSIDE) {
                MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time);
            }
			delay(750);
			MotorStop();
			delay(500);
			return;
		}

		delay(100);
	}
	MotorStop();
}

void MotorRight(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {
	if (Docked == 1) return;
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

        if (CheckSecurity() == SECURITY_BUMPER) {
            CheckChassi();
            return;
        }

		if (HAL_GetTick() - motor_timer >= time_ms) {
			break;
		}

        if (settings.Overturn_Limit == 30 && AbortTurnAndBackIfOutside(RIGHT)) {
            return;
        }
	}

	while (HAL_GetTick() - motor_timer < time_ms || (BWF1_Status == OUTSIDE && HAL_GetTick() - motor_timer < time_ms * 2)) {
        if (CheckSecurity() == SECURITY_BUMPER) {
            CheckChassi();
            return;
        }

        if (settings.Overturn_Limit == 30 && AbortTurnAndBackIfOutside(RIGHT)) {
            return;
        }
	}
	MotorStop();
}

void MotorRight_Guide(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {
    if (Docked == 1) return;
    add_error_event("MotorRight_Guide");
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

        if (HAL_GetTick() - motor_timer >= time_ms) {
            break;
        }
    }

    while (HAL_GetTick() - motor_timer < time_ms) {
        HAL_Delay(1);
    }
    MotorStop();
}

void MotorLeft(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {
	if (Docked == 1) return;
	add_error_event("MotorLeft");
	State = LEFT;
	uint32_t motor_timer;
	motor_timer = HAL_GetTick();


//    uint16_t leftTilt = 0;
//    uint16_t rightTilt = 0;
//
//    if (mpu.roll < 0) {
//        leftTilt = fabs(mpu.roll * settings.roll_tilt_comp);
//    }
//    if (mpu.roll > 0) {
//        rightTilt = fabs(mpu.roll * settings.roll_tilt_comp);
//    }

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

        if (CheckSecurity() == SECURITY_BUMPER) {
            CheckChassi();
            return;
        }

		if (HAL_GetTick() - motor_timer >= time_ms) {
			break;
		}

        if (settings.Overturn_Limit == 30 && AbortTurnAndBackIfOutside(LEFT)) {
            return;
        }
	}
	while (HAL_GetTick() - motor_timer < time_ms || (BWF2_Status == OUTSIDE && HAL_GetTick() - motor_timer < time_ms * 2)) {
        if (CheckSecurity() == SECURITY_BUMPER) {
            CheckChassi();
            return;
        }

        if (settings.Overturn_Limit == 30 && AbortTurnAndBackIfOutside(LEFT)) {
            return;
        }
	}
	MotorStop();
}

void MotorLeft_Guide(uint16_t minSpeed, uint16_t maxSpeed, uint32_t time_ms) {
	if (Docked == 1) return;
	add_error_event("MotorLeft_Guide");
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

		if (HAL_GetTick() - motor_timer >= time_ms) {
			break;
		}
	}
	while (HAL_GetTick() - motor_timer < time_ms) {
        HAL_Delay(1);
    }
	MotorStop();
}

bool AbortTurnAndBackIfOutside(uint8_t dir) {
    CheckSecurity();

    if (dir == LEFT && BWF1_Status == OUTSIDE && BWF2_Status == INSIDE) {
        add_error_event("AbortTurn trg stop on left");

        MotorStop();
        delay(100);
        MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, 400);

        MotorStop();
        return true;
    } else if (dir == RIGHT && BWF1_Status == INSIDE && BWF2_Status == OUTSIDE) {
        add_error_event("AbortTurn trg stop on right");

        MotorStop();
        delay(100);
        MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, 400);

        MotorStop();
        return true;
    } else if (BWF2_Status == OUTSIDE && BWF1_Status == OUTSIDE) {
        MotorStop();
        delay(500);
        if (BWF3_Status == INSIDE) {
            add_error_event("AbortTurn trg backward");
            MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, 2000);
        } else {
            add_error_event("AbortTurn trg stop on both");
        }

        MotorStop();
        return true;
    }

    return false;
}

void MotorStop(void) {
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
	State = BRAKE;

	// Brake - free wheeling
	MOTOR_LEFT_BACKWARD = 0;
	MOTOR_LEFT_FORWARD = 0;
	MOTOR_RIGHT_FORWARD = 0;
	MOTOR_RIGHT_BACKWARD = 0;
}

void MotorHardBrake(void) {
	if (MasterSwitch == 0 || Docked == 1) return;
	State = HARDBRAKE;

	// Wheels will do a hard brake when both pins go HIGH.
	MOTOR_LEFT_BACKWARD = 3359;
	MOTOR_LEFT_FORWARD = 3359;
	MOTOR_RIGHT_FORWARD = 3359;
	MOTOR_RIGHT_BACKWARD = 3359;

	HAL_Delay(250);

	MotorBrake();    //Release motors

}

void CheckState(void) {
	/* This is our main loop function, this is where all the states are
	 * What state is the mower in? and what to do next.
	 */
	if (Initial_Start == 0) {
		return;
	}

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

	if (HAL_GetTick() - OUTSIDE_timer >= (settings.Outside_Threshold * 1000) && Docked == 0) {
		MotorStop();
		cutterOFF();
		return;
	}
	if (bumper_count >= settings.bumper_count_limit) {
		add_error_event("Bumper detection HALT");
		MotorStop();
		MasterSwitch = 0;
		Serial_Console("Bumper detection - HALT\r\n");
		return;
	}

    if (CheckSecurity() == SECURITY_BUMPER) {
        MotorStop();
        return;
    }

    checkIfOverGuideWire();

    if (CheckSecurity() == SECURITY_MOVEMENT) {
		add_error_event("Movement detection HALT");
		MotorStop();
		if (move_count >= settings.move_count_limit) {
			MasterSwitch = 0;
			Serial_Console("Movement detection - HALT\r\n");
			return;
		}
		delay(500);
		MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorBackward_time);
		MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time * 2);
		return;
	}
	if (State == FAIL) {
		add_error_event("State Fail, waiting");
		MotorStop();
		cutterOFF();
		State = STOP;
		return;
	} else if (State == FORWARD && CheckSecurity() == SECURITY_FAIL) {
		add_error_event("FORWARD+SECURITY_FAIL");
		MotorStop();
		getIMUOrientation();
		move_count = 0;
		bumper_count = 0;
		mag_near_bwf = 0;
		highgrass_slowdown = 0;
		TimeToGoHome();            // Check if within working hours, if not, go home
		if (perimeterTracking == 1) {
            trackPerimeter();
            return;
        }
        delay(500);

		CheckSecurity();                // Double check status of the sensors when we are standing still

		if (BWF1_Status == OUTSIDE && BWF2_Status == INSIDE) {
			add_error_event("BWF1 OUT BWF2 IN");
			MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, (settings.motorBackward_time + (fminf(0, mpu.pitch * 50))));
			delay(500);
			MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time + rnd(settings.motorTurnRandom_time));
		} else if (BWF1_Status == INSIDE && BWF2_Status == OUTSIDE) {
			add_error_event("BWF1 IN BWF2 OUT");
			MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, (settings.motorBackward_time + (fminf(0, mpu.pitch * 50))));
			delay(500);
			MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time + rnd(settings.motorTurnRandom_time));
		} else if (BWF1_Status == OUTSIDE && BWF2_Status == OUTSIDE) {
			add_error_event("BWF1 OUT BWF2 OUT");
			MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, (settings.motorBackward_time + (fminf(0, mpu.pitch * 50))));

			delay(500);
            if (BWF1_Status == OUTSIDE && BWF2_Status == OUTSIDE && BWF3_Status == INSIDE) {
                MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorBackward_time * 2);
            }

			if (rnd(100000) < 50000) {
				MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time + rnd(settings.motorTurnRandom_time));
			} else {
				MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time + rnd(settings.motorTurnRandom_time));
			}
		}

		delay(500);
	} else if (guideTrackingActive == 1) {
		MotorStop();
		cutterOFF();
		delay(1000);
		move_count = 0;
		bumper_count = 0;
		mag_near_bwf = 0;
		highgrass_slowdown = 0;
		add_error_event("Found Guide Wire");

		if (settings.use_guide_wire == 1) {
			GoHome_timer_IN = HAL_GetTick();
			GoHome_timer_OUT = HAL_GetTick();

            add_error_event("Try orient on Guide Wire");
            uint16_t speed = round(settings.motorMaxSpeed * 0.84);
            uint32_t startedAt = HAL_GetTick();
            uint8_t initialState = BWF1_guide_status;
            uint8_t inside = OUTSIDE;
            uint8_t outside = INSIDE;
            if (!(BWF1_guide_status == outside && BWF2_guide_status == inside)) {
                CheckBWF();
                while (!(BWF1_guide_status == outside && BWF2_guide_status == inside)) {
                    MOTOR_LEFT_BACKWARD = speed;
                    MOTOR_LEFT_FORWARD = 0;

                    MOTOR_RIGHT_FORWARD = speed;
                    MOTOR_RIGHT_BACKWARD = 0;

                    if (MasterSwitch == 0 || HAL_GetTick() - startedAt > 5750) {
                        add_error_event("Guide orient timeout");
                        break;
                    }

                    HAL_Delay(20);
                    CheckBWF();
                    CheckBWF();
                }
            }

            MotorBrake();

            if (BWF1_guide_status == inside && BWF2_guide_status == outside) {
                add_error_event("Failed guide orient - backwards");
                searchingForGuide = 1;
                guideTrackingActive = 0;
            }
            if (BWF1_guide_status == outside && BWF2_guide_status == inside) {
                add_error_event("Orient guide successful");
            }

			MotorStop();
			GoHome_timer_IN = HAL_GetTick();
			GoHome_timer_OUT = HAL_GetTick();
			return;
		}
	} else if (State == FORWARD) {
		if (MOTOR_LEFT_FORWARD == 0 && MOTOR_RIGHT_FORWARD == 0) {
			State = STOP;
		}
	} else if (State == STOP && CheckSecurity() == SECURITY_OK && Docked == 0) {
		delay(500);
		add_error_event("STOP+SECURITY_OK");
		mpu.hold_heading = mpu.heading;
		if (cutterStatus == 0 && perimeterTracking == 0 && searchingForGuide == 0) {
			cutterON();
		} else if (cutterStatus == 1 && Voltage <= settings.Battery_Low_Limit)
        {
            add_error_event("Cutter off - low bat");
            cutterOFF();
            return;
        }
		mag_near_bwf = 0;
		highgrass_slowdown = 0;
		MotorForward(settings.motorMinSpeed, settings.motorMaxSpeed);
	} else if (State == STOP && CheckSecurity() == SECURITY_FAIL && Docked == 0) {
		delay(500);
		add_error_event("STOP+SECURITY_FAIL");

		CheckSecurity();

        if (Fail_Timer > 0 && (HAL_GetTick() - Fail_Timer > 100000)) {
            MotorBrake();
            cutterOFF();
            add_error_event("Fail timer");
            return;
        }

        uint8_t direction = RIGHT;

		if (BWF1_Status == INSIDE && (BWF2_Status == OUTSIDE || BWF2_Status == NOSIGNAL)) {
            direction = LEFT;
        } else if (BWF2_Status == INSIDE && (BWF1_Status == OUTSIDE || BWF1_Status == NOSIGNAL)) {
            direction = RIGHT;
         } else if (BWF1_Status == OUTSIDE && BWF2_Status == OUTSIDE) {
            if (rnd(100) > 40) {
                direction = RIGHT;
            } else {
                direction = LEFT;
            }
		}

        uint32_t turnTime = settings.motorTurnStatic_time + rnd(settings.motorTurnRandom_time);
        uint32_t backwardsTime = (settings.motorBackward_time + (fmaxf(0, mpu.pitch * 50)));

        if (Fail_Timer > 0 && (HAL_GetTick() - Fail_Timer > 7000)) {
            turnTime *= 2;
            backwardsTime *= 2;
        }

        if (Fail_Timer > 0 && (HAL_GetTick() - Fail_Timer > 15000)) {
            turnTime *= 3;
            backwardsTime *= 3;
        }

        if (Fail_Timer > 0 && (HAL_GetTick() - Fail_Timer > 25000)) {
            if (direction == LEFT) {
                direction = RIGHT;
            } else {
                direction = LEFT;
            }
        }

        if (direction == LEFT) {
            MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, backwardsTime);
            MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, turnTime);
        } else {
            MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, backwardsTime);
            MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, turnTime);
        }

        delay(500);

        if (BWF1_Status == OUTSIDE && BWF2_Status == OUTSIDE && BWF3_Status == INSIDE) {
            MotorBackward(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorBackward_time * 2);
        }
	}
}

void checkIfOverGuideWire() {
    // Check if over guide wire
    if (BWF1_Status == INSIDE && BWF2_Status == INSIDE && searchingForGuide == 1 && guideTrackingActive == 0) {
        uint8_t leftSideTrigger = guideTrackingOut == 0 ? INSIDE : OUTSIDE;
        uint8_t rightSideTrigger = guideTrackingOut == 0 ? OUTSIDE : INSIDE;

        bool crossingWire = (BWF1_guide_status == leftSideTrigger && BWF2_guide_status == rightSideTrigger) || (BWF1_guide_status == rightSideTrigger && BWF2_guide_status == leftSideTrigger);
        bool farFromBoundary = Guide_magBWF1 > magBWF1 && Guide_magBWF2 > magBWF2;
        bool guideMagThreshold = (Guide_magBWF1 >= 175 || Guide_magBWF2 >= 175);
        bool closeToDock = magBWF1 > Guide_magBWF1 || magBWF2 > Guide_magBWF2 || magBWF1 >= settings.magMinValue || magBWF2 >= settings.magMinValue;;

        if (crossingWire && farFromBoundary && guideMagThreshold && !closeToDock) {
            if (Guide_magBWF1 > magBWF1 && Guide_magBWF2 > magBWF2) {
                MotorStop();
                delay(200);
                uint16_t speed  = (settings.motorMaxSpeed + settings.motorMinSpeed) / 2;
                MOTOR_LEFT_BACKWARD = 0;
                MOTOR_LEFT_FORWARD = speed;
                MOTOR_RIGHT_FORWARD = speed;
                MOTOR_RIGHT_BACKWARD = 0;
                delay(800);
                MotorStop();
                delay(100);

                farFromBoundary = Guide_magBWF1 > magBWF1 && Guide_magBWF2 > magBWF2;
                guideMagThreshold = (Guide_magBWF1 >= 175 || Guide_magBWF2 >= 175);
                closeToDock = magBWF1 > Guide_magBWF1 || magBWF2 > Guide_magBWF2 || magBWF1 >= settings.magMinValue || magBWF2 >= settings.magMinValue;;

                delay(100);
                if (Security == INSIDE && farFromBoundary && guideMagThreshold && !closeToDock) {
                    MotorBackward(speed, speed, 700);

                    delay(100);
                    if (Security == INSIDE && guideTrackingOut == 0 && BWF1_guide_status == INSIDE && BWF2_guide_status == OUTSIDE) {
                        MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, 1000);
                    }

                    CheckSecurity();
                    if (Security == INSIDE) {
                        guideTrackingActive = 1;
                        guideTrackingStartedAt = HAL_GetTick();

                        GoHome_timer_OUT = HAL_GetTick();
                        GoHome_timer_IN = HAL_GetTick();
                    }
                } else {
                    MotorBackward(speed, speed, 1000);
                }
            }
        }
    }
}

void trackPerimeter() {
    if (rnd(100) > settings.cut_perimeter_ratio) {
        cutterOFF();
    }
    perimeterTrackingActive = 1;
    delay(500);
    GoHome_timer_IN = HAL_GetTick();
    GoHome_timer_OUT = HAL_GetTick();

    if (BWF1_Status == INSIDE && BWF2_Status == OUTSIDE) {
        return;
    }
    if (BWF1_Status == OUTSIDE && BWF2_Status == OUTSIDE) {
        return;
    }
    if (BWF1_Status == OUTSIDE && BWF2_Status == INSIDE) {
        while (BWF1_Status != INSIDE && BWF2_Status != OUTSIDE) {
            MotorLeft(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time);
            CheckSecurity();
            if (HAL_GetTick() - GoHome_timer_IN >= 10000 || HAL_GetTick() - GoHome_timer_OUT >= 10000) {
                perimeterTracking = 0;
                perimeterTrackingActive = 0;
                MasterSwitch = 0;
                add_error_event("Stuck at turning on perimeter wire");
                break;
            }
        }
        GoHome_timer_IN = HAL_GetTick();
        GoHome_timer_OUT = HAL_GetTick();
        return;
    }

    return;
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
  MX_IWDG_Init();
  MX_RNG_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  	i2c_scanner();

	HAL_TIM_Base_Start(&htim4);                    // Start TIM4 on wheel motors
	HAL_TIM_Base_Start(&htim3);                    // Start TIM3 on cutter motor

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);        //M1 Motor PWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);        //M1 Motor PWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);        //M2 Motor PWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);        //M2 Motor PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);        //C1 Motor PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);        //C1 Motor PWM

	HAL_TIM_Base_Start(&htim5);                    //Start Timer5 for MicroSeconds delay

	MOTOR_LEFT_BACKWARD = 0;                                //M1 Motor - Make sure PWM is 0
	MOTOR_LEFT_FORWARD = 0;                                //M1 Motor - Make sure PWM is 0
	MOTOR_RIGHT_FORWARD = 0;                                //M2 Motor - Make sure PWM is 0
	MOTOR_RIGHT_BACKWARD = 0;                                //M2 Motor - Make sure PWM is 0
	TIM3->CCR1 = 0;                                //C1 Motor - Make sure PWM is 0
	TIM3->CCR2 = 0;                                //C1 Motor - Make sure PWM is 0

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	enable_backup_sram();

	WatchdogInit();                                // STM32 Watchdog - NEVER DISABLE THIS (for safety!)
	HAL_TIM_Base_Start_IT(&htim2);                // 1 second interrupt, will update the watchdog and send info to Serial Console

	Serial_RPi("RazorBoard booting...please wait!\r\n");
	Serial_Console("RazorBoard booting...please wait!\r\n");

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADC_BUFFER, ADC_SAMPLE_LEN);        // Start the DMA for continues scan mode
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *) ADC_REAR_BUFFER, ADC_SAMPLE_LEN);

	InitFIR();                                    // Initiate the FIR functions in hardware

	HAL_RNG_Init(&hrng);                            // Initiate the True Random Number generator

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);    // Enable interrupt for Serial Console
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);    // Enable interrupt for the Raspberry Pi

	ADC_timer = HAL_GetTick();                    // Initial load of the ADC timer
	IMU_timer = HAL_GetTick();                    // Initial load of the IMU timer
	MotorSpeedUpdateFreq_timer = HAL_GetTick();    // Initial load of the MotorSpeedUpdateFreq

	HAL_Delay(5000);

	Init6050();                                    // Start the MPU-6050

	Boundary_Timer = HAL_GetTick();                // Initiate timer for the Boundary Wire

	delay_us(100);

	if (validate_settings(board_revision) == CONFIG_NOT_FOUND) {
		add_error_event("No config found");
		Serial_Console("No config found - Saving factory defaults\r\n");
		Serial_Console("Masterswitch set to OFF - please configure and save settings, then reboot\r\n");
		MasterSwitch = 0;
	}

	settings = read_all_settings();
	Serial_Console("Config loaded from SRAM\r\n");

	for (uint8_t x = 0; x < 60; x++) {
		V1_array[x] = settings.Battery_High_Limit;
	}

	PID.Kp = 20.0;
    PID.Ki = 0.05;
	PID.Kd = 0.02;
//
//    12:50:10: KP: 25.0000
//    12:50:10: KI: 0.0500
//    12:50:10: KD: 0.0200


//    PID.Kp = 20;
//    PID.Ki = 0;
//    PID.Kd = 10;

    arm_pid_init_f32(&PID, 1);

	Serial_RPi("Booting done!\r\n");
	Serial_Console("Booting done!\r\n");
	add_error_event("RazorBoard booted");

	HAL_TIM_Base_Start_IT(&htim6);

	BWF_timer = HAL_GetTick();

	while (1)
	{
		// Collect IMU data every 20 ms, non-blocking.
		if (HAL_GetTick() - IMU_timer >= 20) {
			MPU6050_Read_Accel();
			MPU6050_Read_Gyro();
			ProcessIMUData(settings);
			IMU_timer = HAL_GetTick();
		}

		if (perimeterTrackingActive == 0 && guideTrackingActive == 0 && dockSequenceStartedAt == 0) {
			CheckSecurity();
			CheckState();

			if (State == FORWARD && Force_Active == 1) {
				if (HAL_GetTick() - MotorSpeedUpdateFreq_timer >= MotorSpeedUpdateFreq) {
					UpdateMotorSpeed();
					MotorSpeedUpdateFreq_timer = HAL_GetTick();
				}
			}
		} else {
			if (perimeterTrackingActive == 1) {
				perimeterTracker();
			} else if (guideTrackingActive == 1) {
				PIDtracker();
			}

            if (dockSequenceStartedAt > 0 && HAL_GetTick() - dockSequenceStartedAt > 6000 && Docked == 0) {
                dockSequenceStartedAt = 0;
                if (dockTries > 0 && dockTries <= 3) {
                    add_error_event("Docking sequence expired. Retry.");
                    MotorBackwardImpl(settings.motorMinSpeed, settings.motorMaxSpeed, 2200, true);
                    MotorRight(settings.motorMinSpeed, settings.motorMaxSpeed, settings.motorTurnStatic_time);
                    searchingForGuide = 1;
                    State = STOP;
                } else {
                    add_error_event("Docking sequence expired. Disable.");
                    MotorBackwardImpl(settings.motorMinSpeed, settings.motorMaxSpeed, 300, true);
                    MotorBrake();
                    MasterSwitch = 0;
                    dockTries = 0;
                }
            }
		}

		ChargerConnected();

		if (Docked == 1) {
            dockSequenceStartedAt = 0;
			cutterOFF();
			undock(false);
		}

		HAL_UART_Receive_DMA(&huart1, ConsoleBuffer, CONSOLE_BFR_SIZE);
		HAL_UART_Receive_DMA(&huart2, PIBuffer, PI_BFR_SIZE);

		if (UART1_ready == 1) {
			parseCommand_Console();
		}
		if (UART2_ready == 1) {
			parseCommand_RPI();
		}

		CollectADC();

        if (Collision == 1) CheckChassi();

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
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
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
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 28-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 30000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 1000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : chassiSensor_Pin */
  GPIO_InitStruct.Pin = chassiSensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(chassiSensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : stopButton_Pin */
  GPIO_InitStruct.Pin = stopButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(stopButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : buzzer_Pin */
  GPIO_InitStruct.Pin = buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : voltage_sens_Pin */
  GPIO_InitStruct.Pin = voltage_sens_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(voltage_sens_GPIO_Port, &GPIO_InitStruct);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == chassiSensor_Pin && Docked == 0) {
        bool ignore = State == STOP || State == BRAKE || State == HARDBRAKE;
        if (dockSequenceStartedAt > 0) {
            ignore = false;
        }
        if (ignore && HAL_GPIO_ReadPin(GPIOE, chassiSensor_Pin) == GPIO_PIN_SET) {
            return;
        }

        // Bump
        if (HAL_GPIO_ReadPin(GPIOE, chassiSensor_Pin) == GPIO_PIN_SET) {
            if (Collision == 0)	{
                Collision = 1;
                return;
            }
        }

        // Unbump
        if (HAL_GPIO_ReadPin(GPIOE, chassiSensor_Pin) == GPIO_PIN_RESET) {
            if (Collision == 1)	{
                Collision = 0;
                IsCollisionChecking = 0;
                return;
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	 if(htim->Instance==TIM2) {
		 WatchdogRefresh();        // STM32 Watchdog - NEVER DISABLE THIS (for safety!)
		 SendInfo();
	 }
	 if(htim->Instance==TIM6) {
	 }

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// UART Tx Complete Callback;


	if (huart == &huart1) {
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {                    // Check if it is an "Idle Interrupt"
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);                                // clear the interrupt
			HAL_UART_DMAStop(&huart1);
			UART1_ready = 1;                                                // Serial Console data is now ready to be processed

		}
	}
	if (huart == &huart2) {
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {                    // Check if it is an "Idle Interrupt"
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);                                // clear the interrupt
			HAL_UART_DMAStop(&huart2);
			UART2_ready = 1;                                                // // Raspberry Pi data is now ready to be processed

		}
	}
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
	while (1) {
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
