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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

extern uint8_t UART_Transmit_Done;

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

    delay_us(500);

}
