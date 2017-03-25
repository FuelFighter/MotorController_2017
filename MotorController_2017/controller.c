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