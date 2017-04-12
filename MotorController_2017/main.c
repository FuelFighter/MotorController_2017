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
#include "UniversalModuleDrivers/adc.h"

#define ENCODER_ID 5
#define CRUISECONTROL 2
#define STEERINGWHEEL 20
#define NO_MSG 255
#define INTMAX_CURRENT 0x1FF

#define NORMAL_MODE 0
#define CC_MODE 1



static CanMessage_t canMessage;
static volatile uint8_t newSample = 0;
uint16_t rpm = 0;
uint16_t setPoint_rpm = 0;
uint16_t setPoint_pwm = 0;
uint16_t current = 0;
uint8_t throttle_cmd = 0;
uint8_t state = NORMAL_MODE;
uint8_t cruise_speed = 0;

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
	can_init(0,0);
	timer_init();
	pid_init(0.1, 1.0, 0.0, 2.0);
	adc_init();
	sei();
	
	//setPoint_rpm = 1000;
	
    while (1){	
		can_read_message_if_new(&canMessage);
		/*
		if(canMessage.id == ENCODER_ID){
			cli();
			rpm = (canMessage.data[0] << 8);
			rpm |= canMessage.data[1];
			newSample = 1;
			sei();
		}
		*/
		if (canMessage.id == STEERINGWHEEL){
			
			if((canMessage.data[1]|(CRUISECONTROL << 1)) & (state != CC_MODE)){
				state = CC_MODE;
				cruise_speed = rpm/45; //needs to convert to km/h
				setPoint_rpm = rpm;
				
			}
			else if(canMessage.data[1]|(CRUISECONTROL << 1) ){
			}
		}
		
		switch(state){
			case NORMAL_MODE:
				//Normal stuff
				cli();
				if(canMessage.id == STEERINGWHEEL){
					throttle_cmd = canMessage.data[2];
					setPoint_pwm = throttle_cmd*0xFF;
					
				}
				if(canMessage.id == ENCODER_ID){
					
					rpm = (canMessage.data[0] << 8);
					rpm |= canMessage.data[1];
					newSample = 1;
					
				}
				current_saturation(&setPoint_pwm, &rpm);
				OCR3B = setPoint_pwm;
				
				break;
			case CC_MODE:
			if (canMessage.id == STEERINGWHEEL){
				if (canMessage.data[5] > 75){
					cruise_speed++;
					setPoint_rpm = setPoint_rpm + 45;
					
				}
				if (canMessage.data[5] < 25){
					cruise_speed--;
					setPoint_rpm = setPoint_rpm - 45;
				}
			}
				
			
			if(canMessage.id == ENCODER_ID){
				cli();
				rpm = (canMessage.data[0] << 8);
				rpm |= canMessage.data[1];
				newSample = 1;
				sei();
			}
				//Cruise stuff
				break;
		}
    }
}

ISR(TIMER1_COMPA_vect){
	if (newSample){
		uint16_t pwm_target = controller(rpm,setPoint_rpm);
		current_saturation(&pwm_target, &rpm);
		OCR3B = pwm_target;
		newSample = 0;
		canMessage.id = NO_MSG;
	}
	TCNT1 = 0;
}

