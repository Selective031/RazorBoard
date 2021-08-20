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

UART_HandleTypeDef huart3;

void BLDC_send(char *cmd) {

	char com[16];

    sprintf(com, "%s\r\n", cmd);
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)com, strlen(com));

//    if (strcmp(com, "M1S 0\r\n") == 0)
//    if (strcmp(com, "M2S 0\r\n") == 0)


    HAL_Delay(1);

}
