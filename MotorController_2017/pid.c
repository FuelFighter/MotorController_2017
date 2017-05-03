/*
 * pid.c
 *
 * Created: 24.03.2017 13:52:07
 *  Author: Jorgen Jackwitz
 */ 

#include "pid.h"

#define TIMECONSTANT 1000000
#define GAINCONSTANT 1000
/*
static int lastError = 0;
static int totError = 0;
static int intError = 0;
static int derError = 0;
	   
static int propGain = 0;
static int intGain = 0;
static int derGain = 0;
	 
//static int timeError = 0;
static float timeStep	= 0;
static float Kp = 0;
static float Kd = 0;
static float Ki = 0;


/*
pid_init receives Kp, Kd and Ki as positive floats. TimeStep, t, is a positive float in seconds. 

As long as the setpoint and currentValue are in the same range the pid can calculate for any values between +- 2^15  
*/

/*
void pid_init(float t, float p, float d, float i){
	timeStep = t;
	Kp = p;
	Kd = d;
	Ki = i;
}

int32_t pid(uint16_t currentValue, uint16_t setPoint){

		
		int output = 0;
		printf("currentValue: %u \t",currentValue);
		printf("setPoint: %u \t",setPoint);
		
		totError = setPoint - currentValue;
		
		printf("totError: %d \t", totError);
		
		propGain = totError * Kp;
		
		printf("propGain: %d \t",propGain);
		
		intError = intError + totError*timeStep;
		
		printf("intError: %d \t",intError);
		
		intGain = intError * Ki;
		
		printf("intGain: %d \t",intGain);
		
		derError = (totError - lastError)/timeStep;
		
		lastError = totError;
		
		derGain = derError * Kd;
		
		printf("derGain: %d \t",derGain);
				
		output = propGain + intGain + derGain;
		
		printf("output: %d \n", output);
		
		return output;
}
*/
int32_t pid(Pid_t *PID, uint16_t currentValue, uint16_t setpoint){
	int output = 0;

	PID->totError = setpoint - currentValue;
	// Proportional gain
	int propGain = (PID->Kp)*(PID->totError);
	// Integration gain
	PID->intError = PID->intError + (PID->timeStep)*(PID->totError);
	int intGain = PID->intError*(PID->Ki);
	// Derivation gain
	PID->derError = (PID->totError - PID->lastError)/(PID->timeStep);
	int derGain = (PID->derError)*(PID->Kd);
	PID->lastError = PID->totError;
	/*
	printf("totError: %d \t", PID->totError);
	printf("IntError: %d \t", PID->intError);
	printf("DerError: %d \t",PID->derError);

	printf("PropGain: %d \t", propGain);
	printf("IntGain: %d \t", intGain);
	printf("DerGain: %d \t", derGain);
	*/
	// Pid output
	output = propGain + intGain + derGain;
	//printf("Out: %d \n", output);
	return output;
}

void pid_init(Pid_t *PID, float t, float p, float i, float d){
	PID->Kp = p;
	PID->Ki = i;
	PID->Kd = d;
	PID->timeStep = t;
}