/*
 * PWMtest1803.c
 *
 * Created: 18.03.2017 19:25:19
 * Author : Ultrawack
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "controller.h"
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/pwm.h"
#include "UniversalModuleDrivers/pid.h"
#include "UniversalModuleDrivers/can.h"

#define ENCODER_ID 5

static CanMessage_t canMessage;
static volatile uint8_t newSample = 0;
uint16_t rpm = 0;
uint16_t setPoint = 0;
uint8_t timerInterrupt = 0;

void pin_init(){
	DDRE |= (1<<PE3)|(1<<PE4);
	PORTE &= ~((1<<PE3)|(1<<PE4));
	DDRB &= ~(1<<PB0);
}

void timer_init(){
	TCCR1B |= (1<<CS11)|(1<<CS10);
	TCNT1 = 0;
	TIMSK1 |= (1<<OCIE1A);
	OCR1A = 12500/2;
}

int main(void)
{
	cli();
	pin_init();
	usbdbg_init();
	pwm_init();
	can_init();
	timer_init();
	pid_init(0.1, 1.0, 0.0, 2.0);
	sei();
	
	setPoint = 2500;
	
    while (1){	
		if (can_read_message(&canMessage)){	
			cli();
			rpm = (canMessage.data[0] << 8);
			rpm |= canMessage.data[1];	
			newSample = 1;
			sei();
		}
    }
}

ISR(TIMER1_COMPA_vect){
	timerInterrupt = 1;
	TCNT1 = 0;
}
