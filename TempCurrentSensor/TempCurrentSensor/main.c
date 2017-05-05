	
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
#define LPC (0.1)
#define BIT2MAMP (32.23)

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
	uint16_t adc_val = 0;
	uint16_t prev_adc_val = 0;
	
	while (1) 
    {
		_delay_ms(50);									// 20hz
		printf("ADC: %u\t", adc_read(CH_ADC0));
		uint16_t adc_val = 512-adc_read(CH_ADC0);
		if (adc_val > 1025){
			adc_val = 0;
		}
		adc_val = LPC*adc_val + (1-LPC)*prev_adc_val;
		prev_adc_val = adc_val;
		mamp = BIT2MAMP*adc_val;
		
		canMessage.data[0] = (mamp >> 8);
		canMessage.data[1] = mamp;
		
		printf("mamp: %u \t MSB: %x \t LSB: %x \n", mamp, canMessage.data[0], canMessage.data[1]);
		can_send_message(&canMessage);
	}
}