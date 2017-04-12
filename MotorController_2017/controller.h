/*
 * controller.h
 *
 * Created: 18.03.2017 16:06:03
 *  Author: jorgejac
 */ 


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <avr/io.h>

int32_t controller(uint16_t currentRpm, uint16_t setPoint);
uint16_t current_saturation(uint16_t *rpm, uint16_t *wanted_pwm);

#endif /* CONTROLLER_H_ */