/*
 * controller.h
 *
 * Created: 18.03.2017 16:06:03
 *  Author: jorgejac
 */ 


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <avr/io.h>
#include "pid.h"

int32_t controller(Pid_t *PID, uint16_t currentRpm, uint16_t setPoint);
int32_t controller_trq(Pid_t *PID, uint16_t amp, uint16_t amp_sp);
void current_sample(uint32_t *current_cumulative);
uint16_t current_saturation(uint16_t *rpm, uint16_t *wanted_pwm);
uint8_t safe_addition(uint16_t a,int32_t b);
#endif /* CONTROLLER_H_ */