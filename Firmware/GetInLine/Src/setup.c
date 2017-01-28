/*
 File: setup.c
 Part of the 'Get In Line' line follower

 The MIT License (MIT)

 Copyright (c) 2017 Damian Schneider

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 this file contains some functions needed to handle hardware related stuff
 the HAL driver forces us to put some other functions (like ADC stuff) in the main.c
 */

#include "main.h"
#include "globals.h"
#include "stm32f0xx_hal.h"
#include "stm32f030x6.h"
#include "stm32f0xx_hal_gpio.h"
#include "setup.h"

void readSensorValues(void) {
	//switch on the IR-LED of a channel, then read the ADC. Do this for all three channels
	//channel S1 is on the left, S2 is center, S3 is right
	//S1 is PA10, S2 is PA9, S3 is BP1, setting a pin low will switch the IR LED on

	GPIOA->BRR = GPIO_PIN_10; //pin low
	measurementdelay();
	//HAL_Delay(1);
	//short delay while values stabilize
	sensorValues[0] = ADC_readValue();
	GPIOA->BSRR = GPIO_PIN_10; //pin high


	GPIOA->BRR = GPIO_PIN_9; //pin low
	measurementdelay();
	//HAL_Delay(1);
	//short delay while values stabilize
	sensorValues[1] = ADC_readValue();
	GPIOA->BSRR = GPIO_PIN_9; //pin high

	GPIOB->BRR = GPIO_PIN_1; //pin low
	measurementdelay();
	//HAL_Delay(1);
	sensorValues[2] = ADC_readValue();
	GPIOB->BSRR = GPIO_PIN_1; //pin high

	//todo: could switch all LEDs on and take a reading, this may be used to detect crossings

}

void PWM_SetDuty(uint8_t channel, int16_t dutycycle) //set the dutycycle of the PWM channels 1 and 2 for the motors
{
	if(dutycycle > MAXSPEED) dutycycle = MAXSPEED;
	else if (dutycycle <0) dutycycle = 0;
	uint32_t period = dutycycle << 1; //16bit timer running at full period.
	if (channel == 1) {
		TIM3->CCR1 = period;
	} else if (channel == 2) {
		TIM3->CCR2 = period;
	} else if (channel == 3) {
		TIM3->CCR3 = period;
	} else {
		TIM3->CCR4 = dutycycle;
	}
}

void measurementdelay(void)
{
	uint16_t i;
	for(i=0;i<80;i++)
	{
		asm("nop");
	}
}
