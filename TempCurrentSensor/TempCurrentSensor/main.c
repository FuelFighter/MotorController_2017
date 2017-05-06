	
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




static CanMessage_t msg_M_1;
static CanMessage_t msg_M_2;
#define CURRENT_M_1 100
#define CURRENT_M_2 101;
#define LPC (0.1)
#define BIT2MAMP (32.23)

int main(void)
{
	cli();
	usbdbg_init();
	can_init(0,0);
	adc_init();
	sei();
	
	msg_M_1.id = CURRENT_M_1;
	msg_M_2.id = CURRENT_M_2;
	msg_M_1.length = 2;
	msg_M_2.length = 2;
	uint16_t mamp1 = 0;
	uint16_t mamp2 = 0;
	uint16_t adc_m1_ch0 = 0;
	uint16_t adc_m2_ch1 = 0;
	uint16_t prev_adc_m1_ch0 = 0;
	uint16_t prev_adc_m2_ch1 = 0;
	
	while (1) 
    {
		_delay_ms(50);									// 20hz
		
		uint16_t temp_adc_m1_ch0 = 512 - adc_read(CH_ADC0);
		uint16_t temp_adc_m2_ch1 = 512 - adc_read(CH_ADC1);
		if(temp_adc_m1_ch0 > 1024){
			temp_adc_m1_ch0 = 0;
		}
		if(temp_adc_m2_ch1 > 1024){
			temp_adc_m2_ch1 = 0;
		}
		adc_m1_ch0 = LPC*temp_adc_m1_ch0 + (1-LPC)*prev_adc_m1_ch0;
		adc_m2_ch1 = LPC*temp_adc_m2_ch1 + (1-LPC)*prev_adc_m2_ch1;
		
		mamp1 = BIT2MAMP*adc_m1_ch0;
		mamp2 = BIT2MAMP*adc_m2_ch1;
		
		msg_M_1.data[0] = (mamp1 >> 8);
		msg_M_1.data[1] = mamp1;
		msg_M_2.data[0] = (mamp2 >> 8);
		msg_M_2.data[1] = mamp2;
		
		can_send_message(&msg_M_1);
		can_send_message(&msg_M_2);
	}
}