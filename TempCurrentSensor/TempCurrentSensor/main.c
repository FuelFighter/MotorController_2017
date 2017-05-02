/*
 * TempCurrentSensor.c
 *
 * Created: 5/2/2017 7:18:37 PM
 * Author : Ole
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "../../MotorController_2017/UniversalModuleDrivers/adc.h"
#include "../../MotorController_2017/UniversalModuleDrivers/usbdb.h"
#include "../../MotorController_2017/UniversalModuleDrivers/can.h"
#include "../../MotorController_2017/UniversalModuleDrivers/timer.h"

#define BIT2MAMP (32.23)

int main(void)
{
    cli();
	adc_init();
	can_init();
	timer_init();	
	sei();
	
	CanMessage_t txFrame;
	txFrame.id = 1;
	txFrame.length = 2;
	uint16_t adc_val = 0;
	uint32_t mamp = 0;
	
	timer_start(TIMER0);
	
    while (1){
		if (timer_elapsed_ms(10)){
			txFrame.data[0] = ((0xFF << 8)|mamp);
			txFrame.data[1] = 0xFF|mamp;
		}
		adc_val = adc_read(CH_ADC0);
		mamp = BIT2MAMP*adc_val;
    }
}

