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

#define STROM 100

#define NORMAL_MODE 0
#define CC_MODE 1
#define TORQUE_MODE 2
#define TEST_MODE 0xFF
#define BLANK 0xFE

uint8_t state = NORMAL_MODE;

#define BIT2MAMP (32.23)
#define TC (93.4)
#define MAX_MAMP 2000
#define  MAX_RPM 4500
#define PWM_MAX_DUTY_CYCLE_AT_0_RPM 100
#define PWM_MAX_SCALING_RATIO (float) (ICR3-PWM_MAX_DUTY_CYCLE_AT_0_RPM)/MAX_RPM


// Types
CanMessage_t rxFrame;
CanMessage_t txFrame;
Pid_t Speed;
Pid_t Current;

// Physical values
static uint16_t rpm = 0;
static uint32_t mamp = 0;

// Setpoints and commands
static uint16_t setPoint_rpm = 2000;
static uint16_t setPoint_pwm = 0;
static uint16_t setPoint_mamp = 0;
static uint8_t cruise_speed = 0;
static uint8_t throttle_cmd = 0;
static uint16_t pwm_target = 0;
static uint16_t duty_setpoint = 0;

// Control values
static uint8_t count_speed = 0;
static volatile uint8_t newSample = 0;
static uint16_t nTimerInterrupts = 0;
static uint32_t prev_adc_read = 0;
static uint8_t motor_status = 0b00000000; //(alive|currentoverload|etc|etc|etc|etc|etc|etc|)
static uint8_t send_can = 0;
static uint8_t overload = 1;
static float pwm_top = 0x64;
static uint8_t BMS_status;

void timer_init_ts(){
	TCCR1B |= (1<<CS10)|(1<<CS11);
	TCCR1B |= (1<<WGM12); //CTC
	TCNT1 = 0;
	TIMSK1 |= (1<<OCIE1A);
	OCR1A = 12500 - 1;
}

/*
void handle_can_msg(CanMessage_t *can_msg){
	if(can_read_message_if_new(can_msg)){
		
	}
}
*/

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
	
	DDRB |= (1 << PB4);
	
    while (overload){
		printf("RATIO: %u\n",PWM_MAX_SCALING_RATIO);
		//printf("HEI");
		if (send_can){
			txFrame.data[0] = motor_status;
			txFrame.data[1] = mamp << 8;
			txFrame.data[2] = mamp & 0x00FF;
			txFrame.data[3] = OCR3B << 8;
			txFrame.data[4] = OCR3B & 0x00FF;
			txFrame.data[5] = throttle_cmd;
			txFrame.data[6] = BMS_status;
			can_send_message(&txFrame);
			send_can = 0;
		}
		
		
		
		switch(state){
			case NORMAL_MODE:
				
				if (can_read_message_if_new(&rxFrame)){
					if (rxFrame.id == BMS_STATUS_CAN_ID){
						BMS_status = rxFrame.data[0];
					}
					if(rxFrame.id == STEERING_WHEEL_CAN_ID){
						throttle_cmd = rxFrame.data[3];
						//printf("Thrl: %u\n", throttle_cmd);
					}
					if(rxFrame.id == ENCODER_CAN_ID){
						rpm = (rxFrame.data[0] << 8);
						rpm |= rxFrame.data[1];
						//printf("\tRPM: %u\n", rpm);
					}
					if(rxFrame.id == STROM){
						mamp = (rxFrame.data[0] << 8);
						mamp |= rxFrame.data[1];
						//printf("\t\tMAMP: %u\t\t\n", mamp);
					}
				}
				if(BMS_status == 0x2){
					PORTB &= ~(1 << PB4);
					duty_setpoint = throttle_cmd*(PWM_MAX_DUTY_CYCLE_AT_0_RPM + PWM_MAX_SCALING_RATIO*rpm)*0.01;
					//printf("PWM: %u\t RPM: %u \n ",duty_setpoint, rpm);
					if (duty_setpoint > 719){
						OCR3B = 719;
					}else{
						OCR3B = duty_setpoint;
					}
				}else{
					PORTB |= (1 << PB4);
				}
				break;
				
			case CC_MODE:
			if (rxFrame.id == STEERING_WHEEL_CAN_ID){
				if (rxFrame.data[5] > 75){
					cruise_speed++;
					setPoint_rpm = setPoint_rpm + 45;
				}
				if (rxFrame.data[5] < 25){
					cruise_speed--;
					setPoint_rpm = setPoint_rpm - 45;
				}
			}
				
			
			if(rxFrame.id == ENCODER_CAN_ID){
				cli();
				rpm = (rxFrame.data[0] << 8);
				rpm |= rxFrame.data[1];
				sei();
			}
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
	//printf("RPM: %u \t MAMP: %u \t THRL: %u \t PWM_TOP: %u \n", rpm, mamp, throttle_cmd, pwm_top);
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
/*
ISR(TIMER3_OVF_vect)
{
	if (nTimerInterrupts % 200 == 0)
	{
		_delay_us(10);
		mamp = adc_read(CH_ADC3);
		printf("Amp: %u \r\n", mamp);
		nTimerInterrupts=0;
	}
	nTimerInterrupts++;
}
*/
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

