/*
 * controller.c
 *
 * Created: 18.03.2017 13:30:14
 *  Author: jorgejac
 */ 

#include <avr/io.h>

#define RPMTOPROMILLE 0.2
#define TIMESTEP 0.065536

static uint8_t lastError = 0;
static uint8_t error = 0;
static int intError = 0;
static int derError = 0;

static uint8_t K_p = 5;
static uint8_t K_i = 1;
static uint8_t K_d = 0;

static int propGain = 0;
static int intGain = 0;
static int derGain = 0;

uint16_t controller(int currentRpm, uint16_t setPoint){
	
	uint16_t dutyCycle = 0;
	
	//calculate an percent error
	error = setPoint - currentRpm * RPMTOPROMILLE;
	
	//Calculate proportional gain
	propGain = error * K_p;
	
	//Calculate integral error
	intError = (intError + (error * TIMESTEP));
	
	//Calculate integral gain
	intGain = intError * K_i; 
	
	//Calculate derivate error
	derError = (error - lastError)/TIMESTEP;
	
	//Calculate derivate gain
	derGain = derError * K_d;	
	
	dutyCycle = propGain + intGain + derGain; 
	
	if (dutyCycle >= 1000)
	{
		dutyCycle = 1000;
	} else if (dutyCycle <= 0)
	{
		dutyCycle = 0;
	}
	
	return dutyCycle;
}