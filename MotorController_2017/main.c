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

#define ENCODER_ID 0x005
#define CRUISECONTROL 2
#define STEERINGWHEEL 0x014
#define NO_MSG 255
#define INTMAX_CURRENT 0x1FF

#define NORMAL_MODE 0
#define CC_MODE 1



CanMessage_t rxFrame;
CanMessage_t txFrame;

static volatile uint8_t newSample = 0;
uint16_t rpm = 0;
uint16_t setPoint_rpm = 0;
uint16_t setPoint_pwm = 0;
uint16_t current = 0;
uint8_t throttle_cmd = 0;
uint8_t state = NORMAL_MODE;
uint8_t cruise_speed = 0;

void timer_init(){
	TCCR1B |= (1<<CS11)|(1<<CS10);
	TCNT1 = 0;
	TIMSK1 |= (1<<OCIE1A);
	OCR1A = 12500;
}

int main(void)
{
	cli();
	usbdbg_init();
	pwm_init();
	can_init(0,0);
	timer_init();
	sei();
	
	
    while (1){	
		switch(state){
			case NORMAL_MODE:
				if (can_read_message_if_new(&rxFrame))
				{
					if(rxFrame.id == STEERINGWHEEL){
						throttle_cmd = rxFrame.data[3];
						setPoint_pwm = throttle_cmd*2.55;
					}
					if(rxFrame.id == ENCODER_ID){
						rpm = (rxFrame.data[0] << 8);
						rpm |= rxFrame.data[1];
					}
					printf("Throttle: %u \t",throttle_cmd);
					printf("PWM setpoint: %u \t", setPoint_pwm);
					printf("RPM: %u\n",rpm);
				}
				
				//current_saturation(&setPoint_pwm, &rpm);
				OCR3B = setPoint_pwm;
				
				break;
				
			case CC_MODE:
			if (rxFrame.id == STEERINGWHEEL){
				if (rxFrame.data[5] > 75){
					cruise_speed++;
					setPoint_rpm = setPoint_rpm + 45;
					
				}
				if (rxFrame.data[5] < 25){
					cruise_speed--;
					setPoint_rpm = setPoint_rpm - 45;
				}
			}
				
			
			if(rxFrame.id == ENCODER_ID){
				cli();
				rpm = (rxFrame.data[0] << 8);
				rpm |= rxFrame.data[1];
				//newSample = 1;
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
		//OCR3B = pwm_target;
		newSample = 0;
		rxFrame.id = NO_MSG;
	}
	//OCR3B = setPoint_pwm;
	TCNT1 = 0;
}

