/*
 * controller.c
 *
 * Created: 18.03.2017 13:30:14
 *  Author: jorgejac
 */ 

#include <avr/io.h>
#include "usbdb.h"

#define RPMTOPROMILLE 0.2
#define TIMESTEP 0.1
#define PROMILLETO8BITHEX 0.255

static uint8_t lastError = 0;
static int error = 0;
static int intError = 0;
static int derError = 0;

static int K_p = 1;
static int K_i = 0;
static int K_d = 0;

static int propGain = 0;
static int intGain = 0;
static int derGain = 0;

uint16_t controller(uint16_t currentRpm, uint16_t setPoint){
	
	int output = 0;
	
	//calculate an percent error
	int rpmInPromille = currentRpm * RPMTOPROMILLE;
	
	error = setPoint - rpmInPromille;
	
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
	
	output = propGain + intGain + derGain;
	
	int dutyCycle = output * PROMILLETO8BITHEX; 
	
	if (dutyCycle >= 0xFF)
	{
		dutyCycle = 0xFF;
	} else if (dutyCycle <= 0){
		dutyCycle = 0;
	}
	
	return dutyCycle;
}