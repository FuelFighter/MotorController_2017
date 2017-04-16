/*
 * pid.h
 *
 * Created: 24.03.2017 15:01:25
 *  Author: Ultrawack
 */ 


#ifndef PID_H_
#define PID_H_

#include <avr/io.h>
#include <stdint.h>
#include "usbdb.h"

typedef struct{
	
	int lastError;
	int totError;
	int intError;
	int derError;

	float timeStep;
	float Kp;
	float Kd;
	float Ki;
} Pid_t;

void pid_init(float t, float p, float d, float i);

int32_t pid(int16_t currentValue, int16_t setPoint);

void pid_init_test(Pid_t *PID, float t, float p, float i, float d);

int32_t pid_test(Pid_t *PID, uint16_t currentValue, uint16_t setpoint);

#endif /* PID_H_ */