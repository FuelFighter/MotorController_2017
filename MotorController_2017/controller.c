/*
 * controller.c
 *
 * Created: 18.03.2017 13:30:14
 *  Author: jorgejac
 */ 

#include <avr/io.h>
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/adc.h"
#include "motor_controller_selection.h"
#include "pid.h"

#define RPMTO8BIT 0.051

#define TC 94			//Torque constant
#define SG (0.666)		//Speed/torque gradient
#define SC 102			//Speed constant
#define IMAX 20
#define VCC 50
#define V2PWM 0xFF/VCC

int32_t controller(Pid_t *PID, uint16_t currentRpm, uint16_t setPoint)
{
	int32_t output = pid(PID ,currentRpm, setPoint);	
	
	int32_t dutyCycle = output * RPMTO8BIT; 
	
	if (dutyCycle >= 0xFF)
	{
		dutyCycle = 0xFF;
	} else if (dutyCycle <= 0){
		dutyCycle = 0;
	}
	//printf("DC: %u\n",dutyCycle);
	return dutyCycle;
}

int32_t controller_current(Pid_t *PID, uint16_t amp, uint16_t amp_sp){
	int32_t pid_out = pid(PID, amp, amp_sp);
	return pid_out;
}


int32_t controller_trq(Pid_t *PID, uint16_t amp, uint16_t amp_sp, int32_t *out_last){
	int32_t out = 0;
	
	if ((amp_sp - amp) > 0){
	
	out = pid(PID, amp, amp_sp);
	}else{
		out = *out_last;
	}
	//printf("Out: %u\n",out);
	
	return out;
}

void current_saturation(uint16_t *rpm, uint16_t *pwm){
	int pwmMax = (*rpm + TC*SG*IMAX)/SC;
	printf("RPM: %u \t",*rpm);
	printf("PWMMAX: %u \t",pwmMax);
	if(*pwm > pwmMax){
		*pwm = pwmMax;
	}
}

