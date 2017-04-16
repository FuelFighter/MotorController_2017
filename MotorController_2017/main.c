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
#define STEERINGWHEEL 0x020
#define INTMAX_CURRENT 0x1FF
#define NO_MSG 0xFFF

#define NORMAL_MODE 0
#define CC_MODE 1
#define TORQUE_MODE 2
#define TEST_MODE 0xFF
#define BLANK 0xFE

uint8_t state = BLANK;


// Types
CanMessage_t rxFrame;
CanMessage_t txFrame;
Pid_t Speed;
Pid_t Torque;

// Physical values
static volatile uint8_t newSample = 0;
static uint16_t rpm = 0;
static uint16_t current = 0;

// Setpoints and commands
static uint16_t setPoint_rpm = 2000;
static uint16_t setPoint_pwm = 0;
static uint8_t cruise_speed = 0;
static uint8_t throttle_cmd = 0;

// Control values
static uint8_t count_torque = 0;
static uint8_t cal_torque = 0;




void timer_init_ts(){
	TCCR1B |= (1<<CS10)|(1<<CS11);
	TCCR1B |= (1<<WGM12); //CTC
	TCNT1 = 0;
	TIMSK1 |= (1<<OCIE1A);
	OCR1A = 12500/2 - 1;
}

int main(void)	
{
	cli();
	pid_init_test(&Speed, 1, 1.0, 0.0, 1.0);
	pid_init_test(&Torque, 0.1, 1.0, 0.0, 0.0);
	usbdbg_init();
	pwm_init();
	can_init(0,0);
	timer_init_ts();
	adc_init();
	sei();
	
	//Variables
	uint16_t ampere = 0;				//mV
	
	OCR3B =0xFF;
	
    while (1){
		switch(state){
			case NORMAL_MODE:
				if(cal_torque){
					cli();
					current = adc_read(CH_ADC0);
					current -= 785;
					if (current > 6520)
					{
						current = 0;
					}
					ampere = 7*current;
					cal_torque = 0;
					sei();
				}
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
				}
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
				sei();
			}
				break;
				
			case TORQUE_MODE:
				current = adc_read(CH_ADC0);
				printf("ADC: %u\n",current);
				break;
			case TEST_MODE:
				if (can_read_message_if_new(&rxFrame))
				{
					if (rxFrame.id == ENCODER_ID){
						rpm = (rxFrame.data[0] << 8);
						rpm |= rxFrame.data[1];
						rxFrame.id = NO_MSG;
						newSample = 1;
					}
				}
				break;
				
			case BLANK:
				//Timer test
				break;
		}
    }
}

ISR(TIMER1_COMPA_vect){
	//Torque control goes here 10Hz
	if (newSample){
		uint16_t pwm_target = controller(&Speed, rpm, setPoint_rpm);
		OCR3B = pwm_target;
		newSample = 0;
		rxFrame.id = NO_MSG;
	}
	
	count_torque +=1;
	if (count_torque == 10){
		// Speed Control goes here 1Hz
		count_torque = 0;
	}
	//OCR3B = setPoint_pwm;
	
}
