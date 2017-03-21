/*
 * controller.h
 *
 * Created: 18.03.2017 16:06:03
 *  Author: jorgejac
 */ 


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <avr/io.h>

uint16_t controller(int currentRpm, uint16_t setPoint);


#endif /* CONTROLLER_H_ */