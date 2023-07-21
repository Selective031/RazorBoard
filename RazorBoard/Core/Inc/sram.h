/*
 * sram.h
 *
 *  Created on: 13 Apr 2021
 *      Author: Carl Wallmark
 */

#ifndef INC_SRAM_H_
#define INC_SRAM_H_

static const char VERSION[] = "Version 1.0.8";
static const uint8_t global_settings_version = 46;

#define CONFIG_SET_ADDR					0x01	//uint8_t
#define GO_GOME_DIRECTION_ADDR			0x02	//uint8_t
#define BOUNDARY_TIMEOUT_ADDR			0x03	//uint8_t
#define WORKINGHOURSTART_ADDR			0x04	//uint8_t
#define WORKINGHOUREND_ADDR				0x05	//uint8_t
#define OVERTURN_LIMIT_ADDR				0x06	//uint8_t
#define MOTORSPEEDUPDATEFREQ_ADDR		0x07	//uint8_t
#define OUTSIDE_THRESHOLD_ADDR			0x08	//uint8_t
#define MOVE_COUNT_ADDR					0x09	//uint8_t
#define BUMPER_COUNT_ADDR				0x0A	//uint8_t
#define UNDOCK_BACKING_SECONDS_ADDR		0x0B	//uint8_t
#define CUT_PERIMETER_RATIO_ADDR		0x0C	//uint8_t
#define USE_GUIDE_WIRE_ADDR             0x0D	//uint8_t
#define MULTI_MODE_ADDR                 0x0E	//uint8_t
#define BWF_FLIP_ADDR                   0x0F	//uint8_t

#define HOLDCHARGEDETECTION_ADDR		0x32	//uint16_t
#define MAGVALUE_ADDR					0x34	//uint16_t
#define MAGMINVALUE_ADDR				0x36	//uint16_t
#define MOTORMAXSPEED_ADDR				0x38	//uint16_t
#define MOTORMINSPEED_ADDR				0x3A	//uint16_t
#define CUTTERSPEED_ADDR				0x3C	//uint16_t
#define ADC_LEVEL_ADDR					0x3E	//uint16_t
#define BATTERYCHARGETIME_ADDR			0x40	//uint16_t
#define PERIMETERTRACKERSPEED_ADDR		0x42	//uint16_t
#define ROLL_TILT_COMP_ADDR				0x44	//uint16_t
#define STEERING_CORRECTION_ADDR        0x46	//uint16_t
#define MOTOR_TURN_STATIC_TIME_ADDR		0x48	//uint16_t
#define MOTOR_TURN_RANDOM_TIME_ADDR		0x4A	//uint16_t
#define MOTOR_BACKWARD_TIME_ADDR		0x4C	//uint16_t
#define LAB_1_ADDR              		0x4E	//uint16_t
#define LAB_2_ADDR              		0x50	//uint16_t
#define LAB_3_ADDR              		0x52	//uint16_t
#define MOTOR_CRUISE_SPEED_ADDR 		0x54	//uint16_t
#define LAB_4_ADDR              		0x56	//uint16_t
#define LAB_5_ADDR              		0x58	//uint16_t
#define LAB_6_ADDR              		0x5A	//uint16_t

#define BATTERY_LOW_LIMIT_ADDR			0x64	//uint32_t
#define BATTERY_HIGH_LIMIT_ADDR			0x68	//uint32_t
#define SIGNAL_INTEGRITY_IN_ADDR		0x6C	//uint32_t
#define SIGNAL_INTEGRITY_OUT_ADDR		0x70	//uint32_t
#define MOTOR_LIMIT_ADDR				0x74	//uint32_t
#define CUTTER_LIMIT_ADDR				0x78	//uint32_t
#define KP_ADDR							0x7C	//uint32_t
#define KI_ADDR							0x80	//uint32_t
#define KD_ADDR							0x84	//uint32_t
#define MOVEMENT_ADDR					0x88	//uint32_t
//										0x8C	//uint32_t
#define Motor_Max_Limit_ADDR			0x90	//uint32_t
#define voltageMultiply_ADDR			0x94	//uint32_t
#define proximitySpeed_ADDR				0x98	//uint32_t
#define Motor_Min_Limit_ADDR			0x9C	//uint32_t
#define ROLL_COMP_ADDR					0xA0
#define PITCH_COMP_ADDR					0xA4
#define HIGHGRASS_LIMIT_ADDR			0xA8
#define GUIDE_INTEGRITY_IN_ADDR			0xAC
#define GUIDE_INTEGRITY_OUT_ADDR		0xB0

#define ERRORLOG_INDEX_ADDR				0x1FF
#define ERRORLOG_ADDR					0x200

typedef struct SRAM_ERROR {

	uint8_t index;
	char elog[20][50];

}sram_error;

typedef struct SRAM {

	uint8_t Config_Set;
	uint8_t Go_Home_Direction;
	uint8_t Boundary_Timeout;
	uint8_t WorkingHourStart;
	uint8_t WorkingHourEnd;
	uint8_t Overturn_Limit;
	uint8_t MotorSpeedUpdateFreq;
	uint8_t Outside_Threshold;
	uint8_t move_count_limit;
	uint8_t bumper_count_limit;
	uint8_t undock_backing_seconds;
    uint8_t cut_perimeter_ratio;
    uint8_t use_guide_wire;
    uint8_t multi_mode;
    uint8_t bwf_flip;
    int8_t bwf1_flip; //Calculated from bwf_flip
    int8_t bwf2_flip; //Calculated from bwf_flip
    int8_t bwf3_flip; //Calculated from bwf_flip
    uint16_t HoldChargeDetection;
	uint16_t magValue;
	uint16_t magMinValue;
    uint16_t motorMaxSpeed;
    uint16_t motorMinSpeed;
	uint16_t cutterSpeed;
	uint16_t adcLevel;
	uint16_t BatteryChargeTime;
	uint16_t perimeterTrackerSpeed;
    uint16_t roll_tilt_comp;
    uint16_t steering_correction;
    uint16_t motorTurnStatic_time;
    uint16_t motorTurnRandom_time;
    uint16_t motorBackward_time;
    uint16_t lab_1;
    uint16_t lab_2;
    uint16_t lab_3;
    uint16_t lab_4;
    uint16_t lab_5;
    uint16_t lab_6;
    uint16_t motorCruiseSpeed;
	float Battery_Low_Limit;
	float Battery_High_Limit;
	float Signal_Integrity_IN;
	float Signal_Integrity_OUT;
	float Motor_Limit;
	float Motor_Max_Limit;
	float Motor_Min_Limit;
	float Cutter_Limit;
	float kp;
	float ki;
	float kd;
	float voltageMultiply;
	float proximitySpeed;
	float movement;
	float roll_comp;
	float pitch_comp;
	float highgrass_Limit;
	float Guide_Integrity_IN;
	float Guide_Integrity_OUT;
} sram_settings;

extern void enable_backup_sram(void);
extern void clear_errors(void);
extern void scroll_error_list(void);
extern void add_error_event(char *errormsg);
extern sram_error read_error_log(void);
extern void write_error_log(sram_error errors);
extern uint8_t read_sram_errorlog(uint16_t addr);
extern void write_sram_errorlog(uint8_t l_data, uint16_t addr);
extern void write_sram_uint8(uint8_t l_data, uint8_t addr);
extern uint8_t read_sram_uint8(uint8_t);
extern void write_sram_uint16(uint16_t l_data, uint8_t addr);
extern uint16_t read_sram_uint16(uint8_t);
extern void write_sram_uint32(uint32_t l_data, uint8_t addr);
extern uint32_t read_sram_uint32(uint8_t);
extern void write_sram_float(float l_data, uint8_t);
extern float read_sram_float(uint8_t);
extern sram_settings read_all_settings(void);
extern void write_all_settings(sram_settings w_settings);
extern void save_default_settings(uint8_t revision);
extern void calculate_bwf_flip(sram_settings* settings);
extern uint8_t validate_settings(uint8_t revision);

#define CONFIG_NOT_FOUND 0
#define CONFIG_FOUND 1


#endif /* INC_SRAM_H_ */
