/*
 * main.c
 *
 * Created: 12.04.2017 14:06:29
 *  Author: Ultrawack
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "UniversalModuleDrivers/pwm.h"
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/can.h"
#include "UniversalModuleDrivers/adc.h"

#define ENCODER_ID 5
#define NO_MSG 0xFF
CanMessage_t rxFrame;


int main(void)
{
	cli();
	//pwm_init();
	
	usbdbg_init();
	can_init(0,0);
	sei();
	
	rxFrame.id = NO_MSG;
	
	uint16_t rpm = 0;
	
	while (1)
	{
		/*
		if (can_read_message_if_new(&rxFrame))
		{
			if (rxFrame.id == ENCODER_ID)
			{
				rpm = rxFrame.data[0] << 8;
				rpm |= rxFrame.data[1];
				rxFrame.id = NO_MSG;
				
				printf("RPM: %u \n\r",rpm);
				printf("PWM val: %u \n\r",OCR3B);
			}
		}
		
		OCR3B++;
		if (OCR3B == 0xFF)
		{
			OCR3B = 0;
		}
		*/
		can_read_message_if_new(&rxFrame);
		printf("Can ID: %u, ", rxFrame.id);
		printf("Can Length %u \r\n", rxFrame.length);
		if (rxFrame.data[0])
		{
			printf(rxFrame.data[0]);
		}
		if (rxFrame.id == ENCODER_ID)
		{
			rpm = rxFrame.data[0] << 8;
			rpm |= rxFrame.data[1];
			rxFrame.id = NO_MSG;
			printf("RPM: %u \r\n", rpm);
		}
		
		_delay_ms(100);
	}
}