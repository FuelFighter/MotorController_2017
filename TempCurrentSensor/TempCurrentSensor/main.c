	
/*
 * Encoder_Test_1603.c
 *
 * Created: 16.03.2017 19:24:56
 * Author : Ultrawack
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include "usbdb.h"
#include "can.h"
#include "adc.h"




static CanMessage_t canMessage;
#define STROM 100
#define LP_CONSTANT (0.1)

int main(void)
{
	cli();
	usbdbg_init();
	can_init(0,0);
	adc_init();
	sei();
	
	canMessage.id = STROM;
	canMessage.length = 2;
	uint16_t mamp = 0;
	uint16_t prev_mamp = 0;
	
	while (1) 
    {
		_delay_ms(50);									// 20hz
		printf("ADC: %u\t", adc_read(CH_ADC0));
		uint16_t temp_mamp = 512-adc_read(CH_ADC0);
		if (temp_mamp > 1025){
			temp_mamp = 0;
		}
		mamp = (1-LP_CONSTANT)*prev_mamp + LP_CONSTANT*temp_mamp;
		canMessage.data[0] = (mamp >> 8);
		canMessage.data[1] = mamp;
		
		printf("mamp: %u\n", mamp);
		can_send_message(&canMessage);
	}
}