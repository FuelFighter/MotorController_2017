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
#include "pid.h"
#include "controller.h"
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/pwm.h"
#include "UniversalModuleDrivers/can.h"
#include "UniversalModuleDrivers/adc.h"
#include "motor_controller_selection.h"

// Change Motor in motor_controller_selection.h



uint8_t mode = NORMAL_MODE;
uint8_t state = IDLE;

// Types
CanMessage_t rxFrame;
CanMessage_t txFrame;
Pid_t Speed;
Pid_t Current;

// Physical values

static uint32_t mamp = 0;

// Setpoints and commands
static uint16_t setPoint_rpm = 2000;
static uint16_t setPoint_pwm = 0;
static uint16_t setPoint_mamp = 0;
static uint8_t cruise_speed = 0;
static uint16_t pwm_target = 0;
static uint16_t duty_setpoint = 0;

// Control values
static uint8_t count_speed = 0;
static volatile uint8_t newSample = 0;
static uint16_t nTimerInterrupts = 0;
static uint32_t prev_adc_read = 0;
static uint8_t motor_status = 0b00000000; //(etc|etc|etc|etc|etc|etc|overload|alive|)
static uint8_t send_can = 0;
static uint8_t overload = 0;

void timer_init_ts(){
	TCCR1B |= (1<<CS10)|(1<<CS11);
	TCCR1B |= (1<<WGM12); //CTC
	TCNT1 = 0;
	TIMSK1 |= (1<<OCIE1A);
	OCR1A = 12500 - 1;
}

typedef struct{
	
	uint8_t BMS_status;
	uint8_t throttle_cmd;
	uint8_t restart_overload;
	uint16_t rpm;
	uint32_t temp_mamp;
	
}SubModuleValues_t;


SubModuleValues_t Values = {0};

{
	.BMS_status = 0,
	.throttle_cmd = 0,
	.restart_overload = 0,
	.rpm = 0,
	.temp_mamp = 0
};

void handle_can(SubModuleValues_t *vals, CanMessage_t *rx){
	if (can_read_message_if_new(rx)){
		//switch
		if (rx->id == BMS_STATUS_CAN_ID){
			printf("Status: %b\n", rx->data[0]);
			vals->BMS_status = rx->data[0];
		}if (rx->id == STEERING_WHEEL_CAN_ID){
			printf("\t thrl: %u\n", rx->data[3]);
			vals->throttle_cmd = rx->data[3];
			vals->restart_overload = rx->data[1];
		}if (rx->id == ENCODER_CAN_ID){
			printf("\t\t rpm: %u\n", rx->data[ENCODER_CHANNEL+1]);
			vals->rpm = (rx->data[ENCODER_CHANNEL] << 8);
			vals->rpm |= rx->data[ENCODER_CHANNEL+1];
			
		}if (rx->id == CURRENT_CAN_ID){
			printf("\t\t\t mamp: %u\n", rx->data[CURRENT_CAN_ID+1]);
			vals->temp_mamp = (rx->data[0] << 8);
			vals->temp_mamp |= rx->data[1];
		}
	}
}

int main(void)	
{
	cli();
	pid_init(&Speed, 1, 1.0, 0.0, 1.0);
	pid_init(&Current, 0.1, 0.07, 0.0001, 0.0000);
	usbdbg_init();
	pwm_init();
	can_init(0,0);
	timer_init_ts();
	adc_init();
	txFrame.id = MOTOR_1_STATUS_CAN_ID;
	txFrame.length = 7;
	sei();
	printf("Starting in Idle \n");
	// Output pin to turn off DCDC
	DDRB |= (1 << PB4);
	
    while (1){
		if (send_can){
			txFrame.data[0] = motor_status;
			txFrame.data[1] = Values.temp_mamp >> 8;
			txFrame.data[2] = Values.temp_mamp & 0x00FF;
			txFrame.data[3] = OCR3B >> 8;
			txFrame.data[4] = OCR3B & 0x00FF;
			txFrame.data[5] = Values.throttle_cmd;
			txFrame.data[6] = Values.BMS_status;
			can_send_message(&txFrame);
			send_can = 0;
		}
				
		switch(mode){
			case NORMAL_MODE:
			
				handle_can(&Values, &rxFrame);
				
				switch (state){
					
					case IDLE:
						
						if(Values.BMS_status == 0x2){
							printf("Switch: Idle -> Running\n");
							PORTB |= (1 << PB4);
							state = RUNNING;
							break;
						}
						
						PORTB &= ~(1 << PB4);
						OCR3B = 0;
						break;
						
					case RUNNING:
						if (Values.temp_mamp > MAX_MAMP){
							printf("Switch: Running -> OverCurrent\n");
							OCR3B = 0;
							motor_status |= 1;
							state = OVERLOAD; 
							break;
						}
						if(Values.BMS_status == 0){
							printf("Switch: Running -> Idle\n");
							OCR3B = 0;
							state = IDLE;
							break;
						}
						duty_setpoint = (Values.throttle_cmd)*(PWM_MAX_DUTY_CYCLE_AT_0_RPM + PWM_MAX_SCALING_RATIO*Values.rpm)*0.01;
						if (duty_setpoint >	719){
							OCR3B = 719;
						}else{
							OCR3B = duty_setpoint;
						}	
						break;
						
					case OVERLOAD:
						if (Values.restart_overload){
							printf("Switch: Overload -> Running\n");
							state = RUNNING;
							break;
						}if (Values.BMS_status == 0){
							printf("Switch: Overload -> Idle\n");
							OCR3B = 0;
							state = IDLE;
						}
						OCR3B = 0;
						break;
				}
				break;
				
			case CC_MODE:
			
				break;
				
			case TORQUE_MODE:
				
				break;
	
			case TEST_MODE:				
				
				break;
				
			case BLANK:
				
				break;
		}
    }
}
/*
ISR(TIMER1_COMPA_vect){
	///////////////////TORQUE CONTROL 10 Hz//////////////////////////
	send_can = 1;
	int add = controller_trq(&Current, mamp, setPoint_mamp);
	if ((OCR3B-add) > 0xFFF){
		OCR3B = 0;
	}else if((OCR3B - add) > 0x319){
		OCR3B = 0x319;
	}else{
	OCR3B -= add;
	}
	////////////////////////////////////////////////////////////
	
	count_speed +=1;
	if (count_speed == 10){
	//////////////////////SPEED CONTROL 1Hz/////////////////////
		if (newSample){
			pwm_target = controller(&Speed, rpm, setPoint_rpm);
			OCR3B = pwm_target;
			newSample = 0;
		}
		count_speed = 0;
	/////////////////////////////////////////////////////////////
	}
}
*/

/* IMPLEMENT THIS TO NEW PCB
ISR(TIMER3_COMPA_vect)
{
	if (nTimerInterrupts % 100== 0){
		uint16_t amp_adc = 0;
		uint16_t amp_adc_temp = 0;
		
		for(int i = 0; i < 4; i++){
			amp_adc_temp = 515-adc_read(CH_ADC3);
			if (amp_adc_temp > 1025){
				amp_adc_temp = 0;
			}
			amp_adc += amp_adc_temp;
		}
		
		amp_adc = amp_adc*0.25;
		//printf("raw: %u \t", amp_adc);
		amp_adc = 0.9*prev_adc_read + 0.1*amp_adc;
		//printf("Prev: %u \t\t",prev_adc_read);
		//printf("ADC: %u \n", amp_adc);
		prev_adc_read = amp_adc;
		//mamp = BIT2MAMP*amp_adc;
		//printf("Amp: %u \n", mamp);
		
		nTimerInterrupts = 0;
	}
	nTimerInterrupts++;
}

*/