/*
 * sram.c
 *
 *  Created on: 13 Apr 2021
 *      Author: Carl Wallmark
 */

#include "main.h"
#include "sram.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>


uint8_t read_sram_uint8(uint8_t addr)
{
   uint8_t i_retval;

  /* Enable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write from specific location of backup SRAM */
  i_retval =  *(uint8_t*) (BKPSRAM_BASE + addr);
  /* Disable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_DISABLE();
  return i_retval;
}

void write_sram_uint8(uint8_t l_data, uint8_t addr)
{
   /* Enable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write on specific location of backup SRAM */
  *(uint8_t *) (BKPSRAM_BASE + addr) = l_data;
 /* Disable clock to BKPSRAM */
 __HAL_RCC_BKPSRAM_CLK_DISABLE();
}

uint16_t read_sram_uint16(uint8_t addr)
{
   uint16_t i_retval;

  /* Enable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write from specific location of backup SRAM */
  i_retval =  *(uint16_t*) (BKPSRAM_BASE + addr);
  /* Disable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_DISABLE();
  return i_retval;
}

void write_sram_uint16(uint16_t l_data, uint8_t addr)
{
   /* Enable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write on specific location of backup SRAM */
  *(uint16_t *) (BKPSRAM_BASE + addr) = l_data;
 /* Disable clock to BKPSRAM */
 __HAL_RCC_BKPSRAM_CLK_DISABLE();
}

uint32_t read_sram_uint32(uint8_t addr)
{
   uint32_t i_retval;

  /* Enable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write from specific location of backup SRAM */
  i_retval =  *(uint32_t*) (BKPSRAM_BASE + addr);
  /* Disable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_DISABLE();
  return i_retval;
}

void write_sram_uint32(uint32_t l_data, uint8_t addr)
{
   /* Enable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write on specific location of backup SRAM */
  *(uint32_t *) (BKPSRAM_BASE + addr) = l_data;
 /* Disable clock to BKPSRAM */
 __HAL_RCC_BKPSRAM_CLK_DISABLE();
}

float read_sram_float(uint8_t addr)
{
   uint32_t i_retval;

  /* Enable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write from specific location of backup SRAM */
  i_retval = *(uint32_t*) (BKPSRAM_BASE + addr);
  /* Disable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_DISABLE();
  return *(float *) &i_retval;
}

void write_sram_float(float l_data, uint8_t addr)
{
   /* Enable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write on specific location of backup SRAM */
  *(uint32_t *) (BKPSRAM_BASE + addr) = *(uint32_t *) &l_data;
 /* Disable clock to BKPSRAM */
 __HAL_RCC_BKPSRAM_CLK_DISABLE();
}

void enable_backup_sram(void)
{
    /*DBP : Enable access to Backup domain */
    HAL_PWR_EnableBkUpAccess();
    /*PWREN : Enable backup domain access  */
    __HAL_RCC_PWR_CLK_ENABLE();
    /*BRE : Enable backup regulator
      BRR : Wait for backup regulator to stabilize */
    HAL_PWREx_EnableBkUpReg();
   /*DBP : Disable access to Backup domain */
//    HAL_PWR_DisableBkUpAccess();
}

sram_settings read_all_settings(void)
{
	sram_settings settings;

	settings.Config_Set = read_sram_uint8(CONFIG_SET_ADDR);
	settings.Go_Home_Direction = read_sram_uint8(GO_GOME_DIRECTION_ADDR);
	settings.Boundary_Timeout = read_sram_uint32(BOUNDARY_TIMEOUT_ADDR);
	settings.WorkingHourStart = read_sram_uint8(WORKINGHOURSTART_ADDR);
	settings.WorkingHourEnd = read_sram_uint32(WORKINGHOUREND_ADDR);
	settings.Overturn_Limit = read_sram_uint8(OVERTURN_LIMIT_ADDR);
	settings.MotorSpeedUpdateFreq = read_sram_uint8(MOTORSPEEDUPDATEFREQ_ADDR);
	settings.Outside_Threshold = read_sram_uint8(OUTSIDE_THRESHOLD_ADDR);
	settings.move_count_limit = read_sram_uint8(MOVE_COUNT_ADDR);
	settings.bumber_count_limit = read_sram_uint8(BUMPER_COUNT_ADDR);

	settings.HoldChargeDetection = read_sram_uint16(HOLDCHARGEDETECTION_ADDR);
	settings.magValue = read_sram_uint16(MAGVALUE_ADDR);
	settings.magMinValue = read_sram_uint16(MAGMINVALUE_ADDR);
	settings.motorMaxSpeed = read_sram_uint16(MOTORMAXSPEED_ADDR);
	settings.motorMinSpeed = read_sram_uint16(MOTORMINSPEED_ADDR);
	settings.cutterSpeed = read_sram_uint16(CUTTERSPEED_ADDR);
	settings.adcLevel = read_sram_uint16(ADC_LEVEL_ADDR);

	settings.Battery_High_Limit = read_sram_float(BATTERY_HIGH_LIMIT_ADDR);
	settings.Battery_Low_Limit = read_sram_float(BATTERY_LOW_LIMIT_ADDR);

	settings.Signal_Integrity_IN = read_sram_float(SIGNAL_INTEGRITY_IN_ADDR);
	settings.Signal_Integrity_OUT = read_sram_float(SIGNAL_INTEGRITY_OUT_ADDR);

	settings.Motor_Limit = read_sram_float(MOTOR_LIMIT_ADDR);
	settings.Cutter_Limit = read_sram_float(CUTTER_LIMIT_ADDR);

	settings.kp = read_sram_float(KP_ADDR);
	settings.ki = read_sram_float(KI_ADDR);
	settings.kd = read_sram_float(KD_ADDR);

	settings.Motor_Max_Limit = read_sram_float(Motor_Max_Limit_ADDR);
	settings.Motor_Min_Limit = read_sram_float(Motor_Min_Limit_ADDR);
	settings.voltageMultiply = read_sram_float(voltageMultiply_ADDR);
	settings.proximitySpeed = read_sram_float(proximitySpeed_ADDR);
	settings.movement = read_sram_float(MOVEMENT_ADDR);


	return settings;
}
void write_all_settings(sram_settings settings)
{

	// uint8_t = 1 byte
	// uint16_t = 2 byte
	//uint32_t = 4 byte
	//float = 4 byte

	// uint8_t
	settings.Config_Set = 42;
	write_sram_uint8(settings.Config_Set, CONFIG_SET_ADDR);
	write_sram_uint8(settings.Go_Home_Direction, GO_GOME_DIRECTION_ADDR);
	write_sram_uint8(settings.Boundary_Timeout, BOUNDARY_TIMEOUT_ADDR);
	write_sram_uint8(settings.WorkingHourStart, WORKINGHOURSTART_ADDR);
	write_sram_uint8(settings.WorkingHourEnd, WORKINGHOUREND_ADDR);
	write_sram_uint8(settings.Overturn_Limit, OVERTURN_LIMIT_ADDR);
	write_sram_uint8(settings.MotorSpeedUpdateFreq, MOTORSPEEDUPDATEFREQ_ADDR);
	write_sram_uint8(settings.Outside_Threshold, OUTSIDE_THRESHOLD_ADDR);
	write_sram_uint8(settings.move_count_limit, MOVE_COUNT_ADDR);
	write_sram_uint8(settings.bumber_count_limit, BUMPER_COUNT_ADDR);


	// uint16_t
	write_sram_uint16(settings.HoldChargeDetection, HOLDCHARGEDETECTION_ADDR);
	write_sram_uint16(settings.magValue, MAGVALUE_ADDR);
	write_sram_uint16(settings.magMinValue, MAGMINVALUE_ADDR);
	write_sram_uint16(settings.motorMaxSpeed, MOTORMAXSPEED_ADDR);
	write_sram_uint16(settings.motorMinSpeed, MOTORMINSPEED_ADDR);
	write_sram_uint16(settings.cutterSpeed, CUTTERSPEED_ADDR);
	write_sram_uint16(settings.adcLevel, ADC_LEVEL_ADDR);


	// uint32_t & float
	write_sram_float(settings.Battery_High_Limit, BATTERY_HIGH_LIMIT_ADDR);
	write_sram_float(settings.Battery_Low_Limit, BATTERY_LOW_LIMIT_ADDR);
	write_sram_float(settings.Signal_Integrity_IN, SIGNAL_INTEGRITY_IN_ADDR);
	write_sram_float(settings.Signal_Integrity_OUT, SIGNAL_INTEGRITY_OUT_ADDR);
	write_sram_float(settings.Motor_Limit, MOTOR_LIMIT_ADDR);
	write_sram_float(settings.Cutter_Limit, CUTTER_LIMIT_ADDR);
	write_sram_float(settings.kp, KP_ADDR);
	write_sram_float(settings.ki, KI_ADDR);
	write_sram_float(settings.kd, KD_ADDR);
	write_sram_float(settings.Motor_Max_Limit, Motor_Max_Limit_ADDR);
	write_sram_float(settings.Motor_Min_Limit, Motor_Min_Limit_ADDR);
	write_sram_float(settings.voltageMultiply, voltageMultiply_ADDR);
	write_sram_float(settings.proximitySpeed, proximitySpeed_ADDR);
	write_sram_float(settings.movement, MOVEMENT_ADDR);

}

void save_default_settings(void) {

	sram_settings settings;

	settings.Config_Set = 42;
	settings.Go_Home_Direction = 0;
	settings.Battery_Low_Limit = 22.00;
	settings.Battery_High_Limit = 25.00;
	settings.Boundary_Timeout = 6;
	settings.Cutter_Limit = 2.0;
	settings.HoldChargeDetection = 350;
	settings.MotorSpeedUpdateFreq = 100;
	settings.Outside_Threshold = 8;
	settings.Motor_Limit = 4.0;
	settings.Overturn_Limit = 35;
	settings.Signal_Integrity_IN = 0.80;
	settings.Signal_Integrity_OUT = -0.80;
	settings.WorkingHourStart = 10;
	settings.WorkingHourEnd = 20;
	settings.kp = 0.12;
	settings.ki = 0.0;
	settings.kd = 0.03;
	settings.Motor_Max_Limit = 0.3;
	settings.Motor_Min_Limit = 0.1;
	settings.magValue = 400;
	settings.magMinValue = 370;
	settings.voltageMultiply = 5.0;
	settings.proximitySpeed = 0.80;
	settings.movement = 0.5;
	settings.motorMaxSpeed = 3360 -1;
	settings.motorMinSpeed = 2000;
	settings.cutterSpeed = 3000;
	settings.adcLevel = 1267;
	settings.move_count_limit = 5;
	settings.bumber_count_limit = 10;

	write_all_settings(settings);

}
