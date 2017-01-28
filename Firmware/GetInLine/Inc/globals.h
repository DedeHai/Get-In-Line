/*
  File: globals.h
  Part of the 'Get In Line' line follower

  The MIT License (MIT)

  Copyright (c) 2017 Damian Schneider

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  this file contains all global variables
 */
#include <stdint.h>

#define MAXSPEED 220 //maximum allowed dutycycle (255 is fully on)

//Temperature sensor raw value at 30 degrees C, VDDA=3.3V
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
//Temperature sensor raw value at 110 degrees C, VDDA=3.3V
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
//Internal voltage reference raw value at 30 degrees C, VDDA=3.3V
#define VREFINT_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7BA))

#define LED_ON GPIOF->BSRR = GPIO_PIN_1 //LED pin high
#define LED_OFF GPIOF->BRR = GPIO_PIN_1 //LED pin high

//line position status
enum {
	POSITION_OK, POSITION_CRITICAL, POSITION_BAD
};

volatile uint16_t Vbat; //battery voltage [mV] read from ADC (voltage is Vbat/2), has to be compared against the internal voltage reference to get the real value
volatile uint16_t sensorValues[3]; //values read from the ADC for the reflective sensors
volatile uint16_t potentiometers[4]; //values read from the ADC for the four potentiometers: P, I, D, Speed
volatile int16_t speed; //current speed setting
volatile uint8_t lineposition_state; //state of the line position
