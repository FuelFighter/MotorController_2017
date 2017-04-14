/*
 * efficiency.c
 *
 * Created: 14.04.2017 12:10:32
 *  Author: Ultrawack
 */ 

#include "motorefficiencies.h"
#define MOTOR_ID 1
#define WHEEL_TO_MOTOR1_RPM 10
#define WHEEL_TO_MOTOR2_RPM 14
#define MAX_TORQUE 100
#define MOTOR_RPM_STEP 50
#define MOTOR_CURRENT_STEP 1

uint16_t efficient_gain(uint16_t rpmWheel, uint8_t desired_torque)
{
	uint16_t motor1_rpm_step = (rpmWheel * WHEEL_TO_MOTOR1_RPM)/MOTOR_RPM_STEP;
	uint16_t motor2_rpm_step = (rpmWheel * WHEEL_TO_MOTOR2_RPM)/MOTOR_RPM_STEP;
	
	uint16_t highest_efficiency = 0;
	uint16_t step_efficiency = 0;
	
	uint16_t motor1_torque_gain = 0;
	uint16_t motor2_torque_gain = 0;	
	
	for (int torque_step = 0; torque_step <= desired_torque; torque_step++)
	{
		step_efficiency = motor1[motor1_rpm_step][torque_step] * torque_step + motor2[motor2_rpm_step][desired_torque-torque_step] * torque_step;
		if (step_efficiency > highest_efficiency)
		{
			highest_efficiency = step_efficiency;
			motor1_torque_gain = torque_step;
			motor2_torque_gain = desired_torque - torque_step;
		}
	}
	
	if (MOTOR_ID == 1)
	{
		return motor1_torque_gain;
	} else if (MOTOR_ID == 2)
	{
		return motor2_torque_gain;
	} else {
		return 0x00;
	}	
		
}