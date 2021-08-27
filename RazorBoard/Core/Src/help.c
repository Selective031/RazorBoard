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
#include "uart.h"

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
    sprintf(msg, "Overturn limit: %d\r\n", settings.Overturn_Limit);
    Serial_Console(msg);
    sprintf(msg, "MotorSpeedUpdateFreq: %d\r\n", settings.MotorSpeedUpdateFreq);
    Serial_Console(msg);
    sprintf(msg, "Outside threshold: %d\r\n", settings.Outside_Threshold);
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
    sprintf(msg, "Steering correction: %d\r\n", settings.steering_correction);
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
    sprintf(msg, "Movement limit: %.2f\r\n", settings.movement);
    Serial_Console(msg);
    sprintf(msg, "Motor Max Speed: %d\r\n", settings.motorMaxSpeed);
    Serial_Console(msg);
    sprintf(msg, "Motor Min Speed: %d\r\n", settings.motorMinSpeed);
    Serial_Console(msg);
    sprintf(msg, "Motor Turn Static Time: %d\r\n", settings.motorTurnStatic_time);
    Serial_Console(msg);
    sprintf(msg, "Motor Turn Random Time: %d\r\n", settings.motorTurnRandom_time);
    Serial_Console(msg);
    sprintf(msg, "Motor Backward Time: %d\r\n", settings.motorBackward_time);
    Serial_Console(msg);
    sprintf(msg, "Perimeter Tracker Speed: %d\r\n", settings.perimeterTrackerSpeed);
    Serial_Console(msg);
    sprintf(msg, "Cut perimeter ratio: %d\r\n", settings.cut_perimeter_ratio);
    Serial_Console(msg);
    sprintf(msg, "Cutter Speed: %d\r\n", settings.cutterSpeed);
    Serial_Console(msg);
    sprintf(msg, "Movement count limit: %d\r\n", settings.move_count_limit);
    Serial_Console(msg);
    sprintf(msg, "Bumper limit: %d\r\n", settings.bumper_count_limit);
    Serial_Console(msg);
    sprintf(msg, "Undock backing seconds: %d\r\n", settings.undock_backing_seconds);
    Serial_Console(msg);
    sprintf(msg, "Roll Compensation: %.2f\r\n", settings.roll_comp);
    Serial_Console(msg);
    sprintf(msg, "Pitch Compensation: %.2f\r\n", settings.pitch_comp);
    Serial_Console(msg);
    sprintf(msg, "HighGrass limit: %.1f\r\n", settings.highgrass_Limit);
    Serial_Console(msg);
    sprintf(msg, "Guide IN: %.2f\r\n", settings.Guide_Integrity_IN);
    Serial_Console(msg);
    sprintf(msg, "Guide OUT: %.2f\r\n", settings.Guide_Integrity_OUT);
    Serial_Console(msg);
    sprintf(msg, "Use Guide: %d\r\n", settings.use_guide_wire);
    Serial_Console(msg);
}

void help(void) {
    Serial_Console("Available commands:\r\n\r\n");
    Serial_Console("HELLO                       - Welcome message\r\n");
    Serial_Console("REBOOT                      - Reboot Razorboard\r\n");
    Serial_Console("DISABLE                     - Disable Razorboard\r\n");
    Serial_Console("ENABLE                      - Enable Razorboard\r\n");
    Serial_Console("VERSION                     - Show version of board\r\n");
    Serial_Console("DEBUG ON                    - Enable debug messages\r\n");
    Serial_Console("DEBUG OFF                   - Disable debug messages\r\n");
    Serial_Console("VOLTAGE                     - Show current voltage\r\n");
    Serial_Console("UPGRADE                     - Enter bootloader\r\n");
    Serial_Console("SHOW SIG                    - Show reference BWF signature\r\n");
    Serial_Console("EXPORT SIG                  - Export reference BWF signature as an array\r\n");
    Serial_Console("RECORD SIG                  - Record a new signature\r\n");
    Serial_Console("SHOW GUIDE                  - Show reference GUIDE signature\r\n");
    Serial_Console("EXPORT GUIDE                - Export reference GUIDE signature as an array\r\n");
    Serial_Console("RECORD GUIDE                - Record a new GUIDE signature\r\n");
    Serial_Console("TEST LEFT MOTOR             - Test left motor (M1)\r\n");
    Serial_Console("TEST RIGHT MOTOR            - Test right motor (M2)\r\n");
    Serial_Console("SHOW CURRENT                - Show current sensors M1, M2, C1\r\n");
    Serial_Console("STOP MOTORS                 - Stop motors\r\n");
    Serial_Console("RUN MOTORS FORWARD          - Run motors forward\r\n");
    Serial_Console("RUN MOTORS REVERSE          - Run motors backward\r\n");
    Serial_Console("SET PROXIMITY SPEED         - Set proximity speed\r\n");
    Serial_Console("SET VOLTAGE MULTIPLY        - Voltage Multiply for calculating voltage\r\n");
    Serial_Console("SET MOTOR MAX LIMIT         - Set Motor MAX Limit in amp\r\n");
    Serial_Console("SET MOTOR MIN LIMIT         - Set Motor MIN Limit in amp\r\n");
    Serial_Console("SET MOTOR MAX SPEED         - Set Motor MAX Speed (25 KHz)\r\n");
    Serial_Console("SET MOTOR MIN SPEED         - Set Motor MIN Speed (25 KHz)\r\n");
    Serial_Console("SET BOUNDARY TIMEOUT        - How many seconds without INSIDE before HALT\r\n");
    Serial_Console("SET OVERTURN LIMIT          - How many degrees it can tilt before HALT\r\n");
    Serial_Console("SET OUTSIDE LIMIT           - How many seconds OUTSIDE before HALT\r\n");
    Serial_Console("SET CHARGE DETECTION        - How many (ms) from detecting charge to STOP\r\n");
    Serial_Console("SET BAT LOW                 - Limit when considering charge needed\r\n");
    Serial_Console("SET BAT HIGH                - Limit when considering battery full\r\n");
    Serial_Console("SET BAT CHARGER TIME        - How many minutes to charge battery\r\n");
    Serial_Console("SET BWF OUT                 - Limit for considering BWF OUT\r\n");
    Serial_Console("SET BWF IN                  - Limit for considering BWF IN\r\n");
    Serial_Console("SET CUTTER LIMIT            - Set Cutter Motor Limit in Amp\r\n");
    Serial_Console("SET GUIDE OUT               - Limit for considering GUIDE OUT\r\n");
    Serial_Console("SET GUIDE IN                - Limit for considering GUIDE IN\r\n");
    Serial_Console("SET MOTOR LIMIT             - Set Motor Limit, in multiply, default = 3.0\r\n");
    Serial_Console("SET MOVEMENT LIMIT          - Set Movement Limit for detecting movement\r\n");
    Serial_Console("SET MOTOR TURN STATIC       - Set Motor Turn Time, static\r\n");
    Serial_Console("SET MOTOR TURN RANDOM       - Set Motor Turn Time, random\r\n");
    Serial_Console("SET MOTOR BACKWARD          - Set Motor Backward Time, static\r\n");
    Serial_Console("SET ADC LEVEL               - Set the ADC level for BWF\r\n");
    Serial_Console("SET CUTTER SPEED            - Set speed of cutter motor\r\n");
    Serial_Console("SET MOVEMENT COUNT LIMIT    - Set limit for movement detection before HALT\r\n");
    Serial_Console("SET BUMPER COUNT LIMIT      - Set limit for bumper detection before HALT\r\n");
    Serial_Console("SET UNDOCK BACKING SECONDS  - Set number of seconds to move backwards when undocking\r\n");
    Serial_Console("SET PITCH COMP              - Compensate pitch if not perfectly leveled\r\n");
    Serial_Console("SET ROLL COMP               - Compensate roll if not perfectly leveled\r\n");
    Serial_Console("SET ROLL TILT COMP          - Compensate wheel power ratio when turning based on roll\r\n");
    Serial_Console("SET STEERING CORRECTION     - Compensate wheel power ratio when drifting\r\n");
    Serial_Console("SET HIGHGRASS LIMIT         - When to trigger High Grass, in Amps\r\n");
    Serial_Console("LOCK DOCKING                - Do NOT allow mower to undock when ready\r\n");
    Serial_Console("UNLOCK DOCKING              - Do allow mower to undock when ready\r\n");
    Serial_Console("SET TIME                    - Set current time for RTC\r\n");
    Serial_Console("SET DATE                    - Set current date for RTC\r\n");
    Serial_Console("                              Date must be set in a special order:\r\n");
    Serial_Console("                              Year Month Day Weekday -> 21 3 31 2 (2 = Tuesday)\r\n");
    Serial_Console("TRACK PERIMETER             - Track perimeter next time it crosses\r\n");
    Serial_Console("TRACK GUIDE                 - Track guide next time it crosses\r\n");
    Serial_Console("SET USE GUIDE WIRE          - Enable this to use Guide Wire to charger\r\n");
    Serial_Console("SET PERIMETER SPEED         - Set track perimeter speed\r\n");
    Serial_Console("SET PERIMETER CUT RATIO     - Set ratio for cutting perimeter wire (0-100)\r\n");
    Serial_Console("SET KP                      - PID Controller KP for Perimeter Tracking\r\n");
    Serial_Console("SET KI                      - PID Controller KI for Perimeter Tracking\r\n");
    Serial_Console("SET KD                      - PID Controller KD for Perimeter Tracking\r\n");
    Serial_Console("SET MAG VALUE               - Set Magnitude value for BWF proximity\r\n");
    Serial_Console("SET MAGMIN VALUE            - Set Magnitude Min value for BWF proximity\r\n");
    Serial_Console("SET WORKING START           - Set Working Hour START\r\n");
    Serial_Console("SET WORKING END             - Set Working Hour END\r\n");
    Serial_Console("\r\n");
    Serial_Console("LOAD CONFIG                 - Load config from SRAM\r\n");
    Serial_Console("SAVE CONFIG                 - Save config to SRAM\r\n");
    Serial_Console("SAVE DEFAULT CONFIG         - Save default config to SRAM\r\n");
    Serial_Console("SHOW CONFIG                 - Show config from SRAM\r\n");
    Serial_Console("SHOW ERRORS                 - Show error log\r\n");
    Serial_Console("CLEAR ERRORS                - Clear error log\r\n");
    Serial_Console("SCAN I2C                    - Scan I2C bus for connected devices\r\n");
}
