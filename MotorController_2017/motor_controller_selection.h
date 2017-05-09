/*
 * motor_controller_selection.h
 *
 * Created: 5/9/2017 6:57:30 PM
 *  Author: Ole
 */ 


#ifndef MOTOR_CONTROLLER_SELECTION_H_
#define MOTOR_CONTROLLER_SELECTION_H_

//#define MOTOR_CONTROLLER_1
#define MOTOR_CONTROLLER_2

#ifdef MOTOR_CONTROLLER_1
#define CURRENT_CAN_ID_SELECTION(for1, for2) (for1)
#define ENCODER_READER_SELECTION(for1, for2) (for1)
#endif

#ifdef MOTOR_CONTROLLER_2
#define CURRENT_CAN_ID_SELECTION(for1, for2) (for2)
#define ENCODER_READER_SELECTION(for1, for2) (for2)
#endif

#define NORMAL_MODE 0
#define CC_MODE 1
#define TORQUE_MODE 2
#define TEST_MODE 0xFF
#define BLANK 0xFE

#define BIT2MAMP (32.23)
#define TC (93.4)
#define MAX_MAMP 2000
#define MAX_RPM 4500
#define PWM_MAX_DUTY_CYCLE_AT_0_RPM 8
#define PWM_MAX_SCALING_RATIO (float) (ICR3-PWM_MAX_DUTY_CYCLE_AT_0_RPM)/MAX_RPM

#endif /* MOTOR_CONTROLLER_SELECTION_H_ */