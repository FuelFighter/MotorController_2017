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

#define ENCODER_ID 544
#define STEERINGWHEEL 0x020
#define INTMAX_CURRENT 0x1FF
#define NO_MSG 0xFFF

#define NORMAL_MODE 0
#define CC_MODE 1
#define TORQUE_MODE 2
#define TEST_MODE 0xFF
#define BLANK 0xFE

uint8_t state = TEST_MODE;

#define BIT2MAMP (32.23)
#define TC (93.4)

// Types
CanMessage_t rxFrame;
CanMessage_t txFrame;
Pid_t Speed;
Pid_t Current;

// Physical values

static uint16_t rpm = 0;
static uint32_t cum_amp = 0;
static uint32_t mamp = 0;
static uint32_t trq = 0;

// Setpoints and commands
static uint16_t setPoint_rpm = 2000;
static uint16_t setPoint_pwm = 0;
static uint16_t setPoint_mamp = 0;
static uint8_t cruise_speed = 0;
static uint8_t throttle_cmd = 0;
static uint16_t pwm_target = 0;

// Control values
static uint8_t count_speed = 0;
static uint8_t current_samps = 0;
static volatile uint8_t newSample = 0;
static uint16_t nTimerInterrupts = 0;
static uint32_t prev_adc_read = 0;
static uint8_t can_status = 0b00000000; //(alive|currentoverload|etc|etc|etc|etc|etc|etc|)


void timer_init_ts(){
	TCCR1B |= (1<<CS10)|(1<<CS11);
	TCCR1B |= (1<<WGM12); //CTC
	TCNT1 = 0;
	TIMSK1 |= (1<<OCIE1A);
	OCR1A = 12500 - 1;
}

int main(void)	
{
	printf("Running!\n");
	cli();
	pid_init_test(&Speed, 1, 1.0, 0.0, 1.0);
	pid_init_test(&Current, 0.1, 0.07, 0.0001, 0.0000);
	usbdbg_init();
	pwm_init();
	can_init(0,0);
	timer_init_ts();
	adc_init();
	txFrame.id = MOTOR_1_STATUS_CAN_ID;
	
	sei();
	//printf("EFF: %u\n", efficiency_area(2000,24,5.353));
	
	
    while (1){
		switch(state){
			case NORMAL_MODE:
				if (can_read_message_if_new(&rxFrame))
				{
					if(rxFrame.id == STEERINGWHEEL){
						throttle_cmd = 100-rxFrame.data[3];
						setPoint_pwm = throttle_cmd*8;
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
				
				break;
				
			case TEST_MODE:
				if (can_read_message_if_new(&rxFrame))
				{
					
					if (rxFrame.id == STEERINGWHEEL){
						throttle_cmd = 100-rxFrame.data[3];
					}
					
					if(rxFrame.id == ENCODER_ID){
						rpm = (rxFrame.data[0] << 8);
						rpm |= rxFrame.data[1];
					}
				}
				
				OCR3B = throttle_cmd*8;
				break;
				
			case BLANK:
				
				break;
		}
    }
}

ISR(TIMER1_COMPA_vect){
	///////////////////TORQUE CONTROL 10 Hz//////////////////////////
	/*
	printf("mamp: %u\t",mamp);
	printf("sp_mamp: %u\t", setPoint_mamp);
	printf("ctrl: %d\t ", controller_trq(&Current, mamp, setPoint_mamp));
	printf("sp_mamp: %u \n", setPoint_mamp);
	*/
	uint16_t volt = (48*OCR3B)/ICR3;
	uint8_t eff = efficiency_area(rpm, volt, mamp);
	//printf("Eff: %u \t", eff);
	int add = controller_trq(&Current, mamp, setPoint_mamp);
	if ((OCR3B-add) > 0xFFF){
		OCR3B = 0;
	}else if((OCR3B - add) > 0x319){
		OCR3B = 0x319;
	}else{
	OCR3B -= add;
	}
	
	//printf("OCR: %u", OCR3B);
	
	////////////////////////////////////////////////////////////
	
	count_speed +=1;
	if (count_speed == 10){
	//////////////////////SPEED CONTROL 1Hz/////////////////////
		//printf("rpm: %u\n", rpm);
		if (newSample){
			pwm_target = controller(&Speed, rpm, setPoint_rpm);
			OCR3B = pwm_target;
			newSample = 0;
			rxFrame.id = NO_MSG;
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
		mamp = BIT2MAMP*amp_adc;
		printf("Amp: %u \n", mamp);
		
		nTimerInterrupts = 0;
	}
	nTimerInterrupts++;
}

