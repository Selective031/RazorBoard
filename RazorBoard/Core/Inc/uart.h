/*
 * uart.h
 *
 *  Created on: 1 Jul 2021
 *      Author: Carl Wallmark
 */

#ifndef INC_UART_H_
#define INC_UART_H_

void Serial_Console(char *msg);
void Serial_DATA(char *msg);
void Serial_RPi(char *msg);
void BLDC_send(char *cmd);
void parseCommand_Raptor(void);



#endif /* INC_UART_H_ */
