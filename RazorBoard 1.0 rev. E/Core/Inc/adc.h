/*
 * adc.h
 *
 *  Created on: 22 Apr 2021
 *      Author: Carl Wallmark
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

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

#endif /* INC_ADC_H_ */
