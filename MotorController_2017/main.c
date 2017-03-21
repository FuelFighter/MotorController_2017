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
#include "UniversalModuleDrivers/can.h"
#include "usbdb.h"


//sets bit "bit" in register "reg"
#define set_bit(reg, bit)		(reg |= (1<<bit))
//same as set_bit only clears all other bits
#define set_bit_ex(reg, bit)	(reg = (1<<bit))
//clears bit "bit" in register "reg"
#define clear_bit(reg, bit)		(reg &=~ (1<<bit))
//tests if bit "bit" is set in register "reg"
#define test_bit(reg, bit)		(reg & (1<<bit))
//toggles bit "bit" in register "reg"
#define toggle_bit(reg, bit)	(reg ^= (1<<bit))

#define ENCODER_ID 5

static CanMessage_t canMessage;
static volatile uint8_t newCan = 0;

void pwm_init(void){
	
	//OC3A & B pin as output
	PORTE &= ~(1<<PE3);
	DDRE |= (1<<PE3);
	PORTE &= ~(1<<PE4);
	DDRE |= (1<<PE4);
	
	//fast pwm, mode 14, TOP at ICR
	set_bit(TCCR3B, WGM33);
	set_bit(TCCR3B, WGM32);
	set_bit(TCCR3A,WGM31);
	clear_bit(TCCR3A,WGM30);

	// Set low on compare match for 3A&B
	set_bit(TCCR3A, COM3A1);
	clear_bit(TCCR3A, COM3A0);
	set_bit(TCCR3A, COM3B1);
	clear_bit(TCCR3A, COM3B0);
	
	//Set prescale clk/256 for timer 3
	clear_bit(TCCR3B, CS32);
	clear_bit(TCCR3B, CS31);
	set_bit(TCCR3B, CS30);
	
	//Set top value for timer 3
	ICR3 = 0xFF;
	
	//Set off 3A&B
	OCR3A = 0x0000;
	OCR3B = 0x0000;
}

void pin_init(){
	DDRE |= (1<<PE3)|(1<<PE4);
	PORTE &= ~((1<<PE3)|(1<<PE4));
	
	DDRB &= ~(1<<PB0);
}

void timer_init(){
	TCCR1B |= (1<<CS11)|(1<<CS10);
	TCNT1 = 0;
	TIMSK1 |= (1<<OCIE1A);
	OCR1A = 12500;
}

int main(void)
{
	cli();
	pin_init();
	usbdbg_init();
	pwm_init();
	can_init();
	timer_init();
	sei();
	
	uint16_t rpm = 0;
	uint16_t setPoint = 500;
	
    while (1) 
    {	
		if (canMessage.id == ENCODER_ID && newCan)
		{	
			cli();
			rpm = (canMessage.data[0] << 8);
			rpm |= canMessage.data[1];
			OCR3A = controller(rpm,setPoint);	
			newCan = 0;
			sei();
		}
		_delay_ms(10);
    }
}

ISR(TIMER1_COMPA_vect){
	if (can_read_message(&canMessage))
	{
		newCan = 1;
	}

	TCNT1 = 0;
}
