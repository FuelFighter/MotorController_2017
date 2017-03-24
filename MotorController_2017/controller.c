/*
 * controller.c
 *
 * Created: 18.03.2017 13:30:14
 *  Author: jorgejac
 */ 

#include <avr/io.h>
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/pid.h"

#define RPMTOPROMILLE 0.2
#define PROMILLETO8BITHEX 0.255

int32_t controller(uint16_t currentRpm, uint16_t setPoint){
	
	int16_t rpmInPromille = currentRpm * RPMTOPROMILLE;
	
	int32_t output = pid(rpmInPromille, setPoint);	
	
	int32_t dutyCycle = output * PROMILLETO8BITHEX; 
	
	if (dutyCycle >= 0xFF)
	{
		dutyCycle = 0xFF;
	} else if (dutyCycle <= 0){
		dutyCycle = 0;
	}
	
	return dutyCycle;
}