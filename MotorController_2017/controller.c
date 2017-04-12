/*
 * controller.c
 *
 * Created: 18.03.2017 13:30:14
 *  Author: jorgejac
 */ 

#include <avr/io.h>
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/pid.h"

#define RPMTO8BIT 0.051

#define TC 38			//Torque constant
#define SG (0.668)		//Speed/torque gradient
#define SC 248			//Speed constant
#define IMAX 2
#define VCC 10
#define V2PWM 0xFFFF/VCC

int32_t controller(uint16_t currentRpm, uint16_t setPoint){
	
	int32_t output = pid(currentRpm, setPoint);	
	
	int32_t dutyCycle = output * RPMTO8BIT; 
	
	if (dutyCycle >= 0xFF)
	{
		dutyCycle = 0xFF;
	} else if (dutyCycle <= 0){
		dutyCycle = 0;
	}
	
	return dutyCycle;
}

void current_saturation(uint16_t *rpm, uint16_t *pwm){
	int pwmMax = V2PWM*(*rpm + TC*SC*IMAX)/SG;
	
	if(*pwm > pwmMax){
		*pwm = pwmMax;
	}
}