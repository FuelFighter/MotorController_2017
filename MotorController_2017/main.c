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
//static uint32_t mamp = 0;

// Setpoints and commands
//static uint16_t setPoint_rpm = 2000;
//static uint16_t setPoint_pwm = 0;
static uint16_t setPoint_mamp = 0;
static uint16_t duty_setpoint = 0;

uint8_t read_current;
// Control values
static uint32_t prev_adc_read = 0;
static uint8_t send_can = 0;



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
	uint8_t braking;
	uint32_t mamp;
	uint8_t motor_status; // [|||||statebit2|statebit1|overload]
	uint8_t deadman;
}ModuleValues_t;


ModuleValues_t ComValues = {
	.BMS_status = 0x2,
	.throttle_cmd = 0,
	.restart_overload = 0,
	.rpm = 0,
	.braking = 0,
	.mamp = 0,
	.motor_status = 0,
	.deadman = 0
};



void toggle_DCDC(uint8_t OnOff){
	if (OnOff){
		PORTB &= ~(1 << PB3);
	}else{
		PORTB |= (1 << PB3);
	}
}

void handle_can(ModuleValues_t *vals, CanMessage_t *rx){
	if (can_read_message_if_new(rx)){
		switch (rx->id){
			case BRAKE_CAN_ID:
				vals->braking = rx->data[0];
				break;
			case BMS_STATUS_CAN_ID:
				//vals->BMS_status = rx->data[0];
				break;
			case STEERING_WHEEL_CAN_ID:
				vals->throttle_cmd = rx->data[3];
				vals->restart_overload = rx->data[1] & JOYSTICKBUTTON;
				vals->deadman = rx->data[2];
				break;
			case ENCODER_CAN_ID:
				vals->rpm = (rx->data[ENCODER_CHANNEL+1] << 8);
				vals->rpm |= rx->data[ENCODER_CHANNEL];
				break;
		}
	}
}

void handle_motor_status_can_msg(uint8_t *send, ModuleValues_t *vals){
	if(*send){
		txFrame.data[0] = vals->motor_status;
		txFrame.data[1] = vals->mamp >> 8;
		txFrame.data[2] = vals->mamp & 0x00FF;
		txFrame.data[3] = OCR3B >> 8;
		txFrame.data[4] = OCR3B & 0x00FF;
		txFrame.data[5] = vals->throttle_cmd;
		can_send_message(&txFrame);
		*send = 0;
	}
}

void handle_current_sensor(uint8_t *calculate_current, uint32_t *current, uint16_t *prev_adc){
	if (*calculate_current){
		uint16_t temp_adc = 512 - adc_read(CH_ADC3);
		if (temp_adc > 1024){
			temp_adc = 0;
		}
		uint16_t adc = LOWPASS_CONSTANT*temp_adc + (1-LOWPASS_CONSTANT)*(*prev_adc);
		*prev_adc = adc;
		*current = BIT2MAMP*adc;
	}
}

int main(void)	
{
	cli();
	pid_init(&Current, 0.1, 0.05, 0, 0);
	usbdbg_init();
	pwm_init();
	can_init(0,0);
	timer_init_ts();
	adc_init();
	txFrame.id = MOTOR_CAN_ID;
	txFrame.length = 6;
	sei();
	
	// Output pin to turn off DCDC
	DDRB |= (1 << PB3);
	toggle_DCDC(OFF);
	
	
	
    while (1){

		handle_current_sensor(&read_current, &(ComValues.mamp), &prev_adc_read);
		handle_motor_status_can_msg(&send_can, &ComValues);
		handle_can(&ComValues, &rxFrame);

		switch(mode){
			case NORMAL_MODE:
				switch (state){
					case IDLE:
						//printf("In case IDLE\n");
						if(ComValues.BMS_status == 0x2){
							toggle_DCDC(ON);
							ComValues.motor_status |= (RUNNING << 1);
							state = RUNNING;
							break;
						}
						

						OCR3B = 0;
						break;
					case RUNNING:
						if (ComValues.mamp > MAX_MAMP){
							OCR3B = 0;
							ComValues.motor_status |= 1;
							ComValues.motor_status |= (OVERLOAD << 1);
							state = OVERLOAD; 
							break;
						}
						if(ComValues.BMS_status == 0){
							OCR3B = 0;
							toggle_DCDC(OFF);
							ComValues.motor_status |= (IDLE << 1);
							state = IDLE;
							break;
						}
						if (ComValues.braking || (ComValues.deadman < 50)){
							OCR3B = 0;
						}else{
							duty_setpoint = (ComValues.throttle_cmd)*(PWM_MAX_DUTY_CYCLE_AT_0_RPM + PWM_MAX_SCALING_RATIO*(ComValues.rpm))*0.01;
							if (duty_setpoint >	799){
								OCR3B = 799;
								}else{
								OCR3B = duty_setpoint;
							}
						}
						break;
						
					case OVERLOAD:
						if (ComValues.restart_overload){
							ComValues.motor_status |= (RUNNING << 1);
							state = RUNNING;
							break;
							}if (ComValues.BMS_status == 0){
							OCR3B = 0;
							toggle_DCDC(OFF);
							ComValues.motor_status |= (IDLE << 1);
							state = IDLE;
							}
							OCR3B = 0;
							break;
						}
						break;
				
			case CC_MODE:
			
				break;
				
			case TORQUE_MODE:
				switch(state){
					case IDLE:
						if(ComValues.BMS_status == 0x2){
							toggle_DCDC(ON);
							state = RUNNING;
							break;
						}
						OCR3B = 0;
					break;
					case RUNNING:
						if (ComValues.mamp > MAX_MAMP){
							OCR3B = 0;
							ComValues.motor_status |= 1;
							state = OVERLOAD;
							break;
						}if(ComValues.BMS_status == 0){
							OCR3B = 0;
							toggle_DCDC(OFF);
							state = IDLE;
							break;
						}
						setPoint_mamp = (ComValues.throttle_cmd)*MAX_MAMP*0.01;
					break;
					case OVERLOAD:
						if (ComValues.restart_overload){
							state = RUNNING;
							break;
							}if (ComValues.BMS_status == 0){
							OCR3B = 0;
							toggle_DCDC(OFF);
							state = IDLE;
						}
						
					break;
				}
				break;
			case TEST_MODE:				
				
				break;
				
			case BLANK:
				
				break;
		}
    }
}



ISR(TIMER1_COMPA_vect){
	send_can = 1;
	read_current = 1;
	
	if ((state == RUNNING) && (mode == TORQUE_MODE) ){
		////////////////////////// Torque control 10 hz /////////////////////
		int add = controller_current(&Current, ComValues.mamp, setPoint_mamp);
		if ((OCR3B - add) > 0xFFF){
			OCR3B = 0;
		}else if ((OCR3B - add) > ICR3){
			OCR3B = ICR3;
		}else{
			OCR3B -= add;
		}
		/////////////////////////////////////////////////////////////////////		
	}
}