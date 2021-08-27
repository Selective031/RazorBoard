/*
 * uart.c
 *
 *  Created on: 1 Jul 2021
 *      Author: Carl Wallmark
 */


#include "main.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include <stdlib.h>
#include <ctype.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

float M1_Raptor_Dist = 0;
float M2_Raptor_Dist = 0;
float M1_Raptor_Amp = 0;
float M2_Raptor_Amp = 0;
float C1_Raptor_Amp = 0;
int M1_Raptor_Ticks = 0;
int M2_Raptor_Ticks = 0;
int C1_Raptor_Ticks = 0;
int M1_Raptor_RPM = 0;
int M2_Raptor_RPM = 0;
int C1_Raptor_RPM = 0;

int RPM_M1 = 0;
int RPM_M2 = 0;
int old_ticks_M1 = 0;
int old_ticks_M2 = 0;

uint32_t raptor_uart_timer = 0;
uint8_t UART3_ready = 0;

void Serial_DATA(char *msg) {

	HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), 100);

}
void Serial_RPi(char *msg) {
// Write to Raspberry PI

	HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), 100);

}
void Serial_Console(char *msg) {
// Write to USB/Serial

	HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), 100);

}

void BLDC_send(char *cmd) {

	char com[16];

    sprintf(com, "%s\r\n", cmd);
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)com, strlen(com));

    delay_us(600);

}
void parseCommand_Raptor(void) {

	char Command[192] = {"\0"};

	for (uint8_t x = 0; x < 192; x++) {
		if (RaptorBuffer[x] == 13) {
			memcpy(Command,RaptorBuffer,x);

			for (uint8_t i = 0; i < RAPTOR_BFR_SIZE; i++) {
				Command[i] = toupper(Command[i]);
			}

			if (strncmp(Command, "M1-DIST", 7) == 0) {
				char cmd1[7], cmd2[6], cmd3[8], cmd4[6],cmd5[7],cmd6[6],cmd7[8],cmd8[6],cmd9[6],cmd10[8],cmd11[6];
				sscanf(Command, "%s %f %s %d %s %d %s %f %s %f %s %d %s %d %s %f %s %d %s %d %s %f", cmd1, &M1_Raptor_Dist, cmd2, &M1_Raptor_RPM, cmd3, &M1_Raptor_Ticks, cmd4, &M1_Raptor_Amp, cmd5, &M2_Raptor_Dist, cmd6, &M2_Raptor_RPM, cmd7, &M2_Raptor_Ticks, cmd8, &M2_Raptor_Amp, cmd9, &C1_Raptor_RPM, cmd10, &C1_Raptor_Ticks, cmd11, &C1_Raptor_Amp);
				memset(RaptorBuffer, 0, sizeof(RAPTOR_BFR_SIZE));

			}

			break;
		}
	}
	memset(RaptorBuffer, 0, sizeof(RAPTOR_BFR_SIZE));
	UART3_ready = 0;

}
