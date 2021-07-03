/*
 * help.c
 *
 *  Created on: 26 Apr 2021
 *      Author: Carl Wallmark
 */

#include "main.h"
#include "sram.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>

void show_error() {

	sram_error errors;
	errors = read_error_log();

	for (int i = 0; i < 20; i++) {

		sprintf(msg, "[%d] %s\r\n", i, errors.elog[i]);
		Serial_Console(msg);

	}
}

void show_config(sram_settings settings) {

	sprintf(msg, "Go Home Direction: %d\r\n", settings.Go_Home_Direction);
	Serial_Console(msg);
	sprintf(msg, "Boundary_Timeout: %d\r\n", settings.Boundary_Timeout);
	Serial_Console(msg);
	sprintf(msg, "WorkingHourStart: %d\r\n", settings.WorkingHourStart);
	Serial_Console(msg);
	sprintf(msg, "WorkingHourEnd: %d\r\n", settings.WorkingHourEnd);
	Serial_Console(msg);
	sprintf(msg, "Overturn_Limit: %d\r\n", settings.Overturn_Limit);
	Serial_Console(msg);
	sprintf(msg, "MotorSpeedUpdateFreq: %d\r\n", settings.MotorSpeedUpdateFreq);
	Serial_Console(msg);
	sprintf(msg, "Outside_Threshold: %d\r\n", settings.Outside_Threshold);
	Serial_Console(msg);
	sprintf(msg, "HoldChargeDetection: %d\r\n", settings.HoldChargeDetection);
	Serial_Console(msg);
	sprintf(msg, "Battery High: %.2f\r\n", settings.Battery_High_Limit);
	Serial_Console(msg);
	sprintf(msg, "Battery Low: %.2f\r\n", settings.Battery_Low_Limit);
	Serial_Console(msg);
	sprintf(msg, "Battery charge time (min): %d\r\n", settings.BatteryChargeTime);
	Serial_Console(msg);
	sprintf(msg, "Signal IN: %.2f\r\n", settings.Signal_Integrity_IN);
	Serial_Console(msg);
	sprintf(msg, "Signal OUT: %.2f\r\n", settings.Signal_Integrity_OUT);
	Serial_Console(msg);
	sprintf(msg, "Motor Limit: %.2f\r\n", settings.Motor_Limit);
	Serial_Console(msg);
	sprintf(msg, "Cutter Limit: %.2f\r\n", settings.Cutter_Limit);
	Serial_Console(msg);
	sprintf(msg, "Motor Max Limit: %.2f\r\n", settings.Motor_Max_Limit);
	Serial_Console(msg);
	sprintf(msg, "Motor Min Limit: %.2f\r\n", settings.Motor_Min_Limit);
	Serial_Console(msg);
	sprintf(msg, "ADC Level: %d\r\n", settings.adcLevel);
    Serial_Console(msg);
    sprintf(msg, "Roll/tilt comp: %d\r\n", settings.roll_tilt_comp);
	Serial_Console(msg);
	sprintf(msg, "KP: %.4f\r\n", settings.kp);
	Serial_Console(msg);
	sprintf(msg, "KI: %.4f\r\n", settings.ki);
	Serial_Console(msg);
	sprintf(msg, "KD: %.4f\r\n", settings.kd);
	Serial_Console(msg);
	sprintf(msg, "Magnitude Proximity: %d\r\n", settings.magValue);
	Serial_Console(msg);
	sprintf(msg, "Magnitude Min Proximity: %d\r\n", settings.magMinValue);
	Serial_Console(msg);
	sprintf(msg, "Voltage Multiply: %.4f\r\n", settings.voltageMultiply);
	Serial_Console(msg);
	sprintf(msg, "Proximity Speed: %.2f\r\n", settings.proximitySpeed);
	Serial_Console(msg);
	sprintf(msg, "Movement Limit: %.2f\r\n", settings.movement);
	Serial_Console(msg);
	sprintf(msg, "Motor Max Speed: %d\r\n", settings.motorMaxSpeed);
	Serial_Console(msg);
	sprintf(msg, "Motor Min Speed: %d\r\n", settings.motorMinSpeed);
	Serial_Console(msg);
	sprintf(msg, "Perimeter Tracker Speed: %d\r\n", settings.perimeterTrackerSpeed);
	Serial_Console(msg);
	sprintf(msg, "Cutter Speed: %d\r\n", settings.cutterSpeed);
    Serial_Console(msg);
    sprintf(msg, "Movement limit: %d\r\n", settings.move_count_limit);
    Serial_Console(msg);
    sprintf(msg, "Bumber limit: %d\r\n", settings.bumber_count_limit);
    Serial_Console(msg);
    sprintf(msg, "Undock backing seconds: %d\r\n", settings.undock_backing_seconds);
    Serial_Console(msg);
	sprintf(msg, "Roll Compensation: %.2f\r\n", settings.roll_comp);
	Serial_Console(msg);
	sprintf(msg, "Pitch Compensation: %.2f\r\n", settings.pitch_comp);
	Serial_Console(msg);
	sprintf(msg, "HighGrass limit: %.1f\r\n", settings.highgrass_Limit);
	Serial_Console(msg);

}

void help(void) {
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
	sprintf(msg, "SET PROXIMITY SPEED     - Set proximity speed\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET VOLTAGE MULTIPLY   - Voltage Multiply for calculating voltage\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET MOTOR MAX LIMIT     - Set Motor Max Limit in amp\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET MOTOR MIN LIMIT     - Set Motor Min Limit in amp\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET BOUNDARY TIMEOUT    - How many seconds without INSIDE before HALT\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET OVERTURN LIMIT      - How many degrees it can tilt before HALT\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET OUTSIDE LIMIT       - How many seconds OUTSIDE before HALT\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET CHARGE DETECTION    - How many (ms) from detecting charge to STOP\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET BAT LOW             - Limit when considering charge needed\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET BAT HIGH            - Limit when considering battery full\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET BAT CHARGER TIME    - How many minutes to charge battery\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET BWF OUT             - Limit for considering BWF OUT\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET BWF IN              - Limit for considering BWF IN\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET CUTTER LIMIT        - Set Cutter Motor Limit in Amp\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET MOTOR LIMIT         - Set Motor Limit, in multiply, default = 3.0\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET MOVEMENT LIMIT      - Set Movement Limit for detecting movement\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET ADC LEVEL           - Set the ADC level for BWF\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET CUTTER SPEED        - Set speed of cutter motor\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET MOVEMENT COUNT LIMIT    - Set limit for movement detection before HALT\r\n");
	Serial_Console(msg);
    sprintf(msg, "SET BUMBER COUNT LIMIT  - Set limit for bumber detection before HALT\r\n");
    Serial_Console(msg);
    sprintf(msg, "SET UNDOCK BACKING SECONDS  - Set number of seconds to move backwards when undocking\r\n");
    Serial_Console(msg);
	sprintf(msg, "SET PITCH COMP          - Compensate pitch if not perfectly leveled\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET ROLL COMP           - Compensate roll if not perfectly leveled\r\n");
    Serial_Console(msg);
    sprintf(msg, "SET ROLL TILT COMP      - Compensate wheel power ratio when turning based on roll\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET HIGHGRASS LIMIT     - When to trigger High Grass, in Amps\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET TIME		- Set current time for RTC\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET DATE		- Set current date for RTC\r\n");
	Serial_Console(msg);
	sprintf(msg, "			Date must be set in a special order:\r\n");
	Serial_Console(msg);
	sprintf(msg, "			Year Month Day Weekday -> 21 3 31 2 (2 = Tuesday)\r\n");
	Serial_Console(msg);
	sprintf(msg, "TRACK PERIMETER 	- Track perimeter next time it crosses\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET PERIMETER SPEED   - Set track perimeter speed\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET KP			- PID Controller KP for Perimeter Tracking\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET KI			- PID Controller KI for Perimeter Tracking\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET KD			- PID Controller KD for Perimeter Tracking\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET MAG VALUE     - Set Magnitude value for BWF proximity\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET MAGMIN VALUE     - Set Magnitude Min value for BWF proximity\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET WORKING START	- Set Working Hour START\r\n");
	Serial_Console(msg);
	sprintf(msg, "SET WORKING END	- Set Working Hour END\r\n");
	Serial_Console(msg);
	Serial_Console("\r\n");
	sprintf(msg, "LOAD CONFIG        - Load config from SRAM\r\n");
	Serial_Console(msg);
	sprintf(msg, "SAVE CONFIG        - Save config to SRAM\r\n");
	Serial_Console(msg);
	sprintf(msg, "SAVE DEFAULT CONFIG     - Save default config to SRAM\r\n");
	Serial_Console(msg);
	sprintf(msg, "SHOW CONFIG        - Show config from SRAM\r\n");
	Serial_Console(msg);
}
