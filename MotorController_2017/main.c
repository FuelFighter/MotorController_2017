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
#include "UniversalModuleDrivers/rgbled.h"
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/pwm.h"
#include "UniversalModuleDrivers/can.h"
#include "UniversalModuleDrivers/adc.h"
#include "motor_controller_selection.h"

// Change Motor in motor_controller_selection.h

uint8_t mode = NORMAL_MODE;
MotorControllerState_t state = IDLE;

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

// Control values
static uint32_t prev_current = 0;
static uint8_t send_can = 0;
static uint8_t read_current = 0;



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
	MotorControllerState_t motor_status; // [||||||statebit2|statebit1]
	uint8_t deadman;
}ModuleValues_t;


ModuleValues_t ComValues = {
	.BMS_status = 0x0,
	.throttle_cmd = 0,
	.restart_overload = 0,
	.rpm = 0,
	.braking = 0,
	.mamp = 0,
	.motor_status = IDLE,
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
				vals->braking = rx->data.u8[0];
				break;
			case BMS_STATUS_CAN_ID:
				vals->BMS_status = rx->data.u8[0];
				break;
			case STEERING_WHEEL_CAN_ID:
				vals->throttle_cmd = rx->data.u8[3];
				vals->restart_overload = rx->data.u8[1] & HORN;
				vals->deadman = rx->data.u8[2];
				break;
			case ENCODER_CAN_ID:
				vals->rpm = rx->data.u16[ENCODER_CHANNEL];
				break;
		}
	}
}

void handle_motor_status_can_msg(uint8_t *send, ModuleValues_t *vals){
	if(*send){
		txFrame.data.u8[0] = vals->motor_status;
		txFrame.data.u8[1] = vals->throttle_cmd;
		txFrame.data.u16[1] = vals->mamp;
		txFrame.data.u16[2] = OCR3B;
		txFrame.data.u16[3] = vals->rpm;
		
		can_send_message(&txFrame);
		*send = 0;
	}
}

void handle_current_sensor(uint8_t *calculate_current, uint32_t *current, uint32_t *prev_current){
	if (*calculate_current){
		uint16_t temp_adc = 512 - adc_read(CH_ADC3);
		if (temp_adc > 1024){
			temp_adc = 0;
		}
		uint16_t mamp = BIT2MAMP*temp_adc;
		*current = LOWPASS_CONSTANT*(mamp) + (1 - LOWPASS_CONSTANT)*(*prev_current);
		*prev_current = *current;
		*calculate_current = 0;
	}
}

int main(void)	
{
	cli();
	pid_init(&Current, 0.1, 0.05, 0, 0);
	usbdbg_init();
	pwm_init();
	pwm_set_top_t3(0x319);
	can_init(0,0);
	timer_init_ts();
	adc_init();
	rgbled_init();
	txFrame.id = MOTOR_CAN_ID;
	txFrame.length = 8;
	sei();
	
	// Output pin to turn off DCDC
	DDRB |= (1 << PB3);
	toggle_DCDC(OFF);
	
	rgbled_turn_on(LED_BLUE);
	
    while (1){

		handle_current_sensor(&read_current, &(ComValues.mamp), &prev_current);
		handle_motor_status_can_msg(&send_can, &ComValues);
		handle_can(&ComValues, &rxFrame);

		switch(mode){
			case NORMAL_MODE:
				switch (state){
					case IDLE:
						//printf("In case IDLE\n");
						if(ComValues.BMS_status == 0x2){
							toggle_DCDC(ON);
							ComValues.motor_status = RUNNING;
							state = RUNNING;
							rgbled_turn_off(LED_ALL);
							rgbled_turn_on(LED_GREEN);
							break;
						}
						OCR3B = 0;
						break;
					case RUNNING:
						if (ComValues.mamp > MAX_MAMP){
							OCR3B = 0;
							ComValues.motor_status |= 1;
							ComValues.motor_status = OVERLOAD;
							rgbled_turn_off(LED_ALL);
							rgbled_turn_on(LED_RED);
							state = OVERLOAD; 
							break;
						}
						if(ComValues.BMS_status == 0){
							OCR3B = 0;
							toggle_DCDC(OFF);
							ComValues.motor_status = IDLE;
							rgbled_turn_off(LED_ALL);
							rgbled_turn_on(LED_BLUE);
							state = IDLE;
							break;
						}
						if (ComValues.braking || (ComValues.deadman < 50)){
							OCR3B = 0;
						}else{
							duty_setpoint = (ComValues.throttle_cmd)*(PWM_MAX_DUTY_CYCLE_AT_0_RPM + PWM_MAX_SCALING_RATIO*(ComValues.rpm))*0.01;
							if (duty_setpoint >	ICR3){
								OCR3B = ICR3;
								}else{
								OCR3B = duty_setpoint;
							}
						}
						break;
						
					case OVERLOAD:
						if (ComValues.restart_overload){
							ComValues.motor_status = RUNNING << 1;
							rgbled_turn_off(LED_ALL);
							rgbled_turn_on(LED_GREEN);
							state = RUNNING;
							break;
							}if (ComValues.BMS_status == 0){
							OCR3B = 0;
							toggle_DCDC(OFF);
							ComValues.motor_status |= (IDLE << 1);
							rgbled_turn_off(LED_ALL);
							rgbled_turn_on(LED_BLUE);
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