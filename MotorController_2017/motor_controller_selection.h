/*
 * motor_controller_selection.h
 *
 * Created: 5/9/2017 6:57:30 PM
 *  Author: Ole
 */ 


#ifndef MOTOR_CONTROLLER_SELECTION_H_
#define MOTOR_CONTROLLER_SELECTION_H_

#include "UniversalModuleDrivers/can.h"

// To choose motor controller, comment out opposite
#define MOTOR_CONTROLLER_1
//#define MOTOR_CONTROLLER_2

#ifdef MOTOR_CONTROLLER_1
#define MOTOR_SELECT(for1, for2) (for1)
#endif

#ifdef MOTOR_CONTROLLER_2
#define MOTOR_SELECT(for1, for2) (for2)
#endif

#define MOTOR_CAN_ID					MOTOR_SELECT(MOTOR_1_STATUS_CAN_ID, MOTOR_2_STATUS_CAN_ID)
#define CURRENT_CAN_ID					MOTOR_SELECT(100, 101)
#define ENCODER_CHANNEL					MOTOR_SELECT(0, 2)

#define NORMAL_MODE						0
#define CC_MODE							1
#define TORQUE_MODE						2
#define TEST_MODE						0xFF
#define BLANK							0xFE

#define MAX_MAMP						MOTOR_SELECT(4500, 6800)
#define MAX_RPM							MOTOR_SELECT(4000, 3500)
#define PWM_MAX_DUTY_CYCLE_AT_0_RPM		160
#define PWM_MAX_SCALING_RATIO (float)	(ICR3 - PWM_MAX_DUTY_CYCLE_AT_0_RPM)/MAX_RPM

#define IDLE 0
#define RUNNING 1
#define OVERLOAD 2

#define OFF 0
#define ON 1

#define HORN 2
#define JOYSTICKBUTTON 1

#define LOWPASS_CONSTANT (0.1)
#define BIT2MAMP (32.23)

#endif /* MOTOR_CONTROLLER_SELECTION_H_ */