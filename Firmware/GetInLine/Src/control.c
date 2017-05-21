/*
 File: control.c
 Part of the 'Get In Line' line follower

 The MIT License (MIT)

 Copyright (c) 2017 Damian Schneider

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 this file contains the algorithms to control the robot
 */
#include "stm32f0xx_hal.h"
#include "globals.h"
#include "setup.h"

#define SENSORSPACING 7500 //sensor x-spacing in micrometers
#define ANTIWINDUP_MAX 900 //maximum allowed value for the integrator

//interpolation algorithm for the three sensor values. line position is micrometers from the center (positive is right, negative left)
int16_t calculateLinePosition(void) {
	int16_t lineposition;
	static int16_t lastlineposition = 5000;
	static int16_t calibrate_countdown = 500; //take 500 measurements to calibrate, then stop

	static uint16_t sensorminvalue[3] = { 2000, 2000, 2000 }; //minimum value seen by each sensor (start with a high value)

	//save the minimum sensorvalues and use them as a sensor calibration bias
	uint8_t i;
	if (calibrate_countdown) //only calibrate at the beginning
	{
		if (sensorValues[0] < 3400 && sensorValues[1] < 3400
				&& sensorValues[2] < 3400) { //if actually on the ground in a whiteish environment
			for (i = 0; i < 3; i++) {
				if (sensorValues[i] < sensorminvalue[i]) {
					sensorminvalue[i] =
							(8 * sensorminvalue[i] + (uint16_t)sensorValues[i]) / 9; //use simple lowpass filter
				}
				//calibrate the sensor value (deduct the minimum value, sensorvalue must never be negative!)
				//sensorValues[i]=((uint32_t)sensorValues[i]<<10)/ sensorminvalue[i];

			}
			calibrate_countdown--;
		}
	}
	//apply calibration data
	for (i = 0; i < 3; i++) {
	sensorValues[i] -= sensorminvalue[i];
	if (sensorValues[i] < 0)
		sensorValues[i] = 0;
	}
	//the line position is the weighted average of the sensor values:
	//weighted with the position, position of the left sensor is taken as 'SENSORSPACING' since 0 is not a good idea

	uint32_t weightedsum = SENSORSPACING * (uint32_t) sensorValues[0]
			+ 2 * SENSORSPACING * (uint32_t) sensorValues[1]
			+ 3 * SENSORSPACING * (uint32_t) sensorValues[2];

	uint32_t weightedavg = weightedsum
			/ ((uint32_t) sensorValues[0] + (uint32_t) sensorValues[1]
					+ (uint32_t) sensorValues[2]);
	//now we have the weighted average of the position but with an offset (zero position is one SENSORSPACING to the left of the left sensor)
	//deduct two SENSORSPACINGS to offset to the center position
	lineposition = (int32_t) weightedavg - 2 * (int32_t) SENSORSPACING;
	/*
	 //if line position error is small, check if line is moving out of sight
	 if (lineposition < 1000 && lineposition > -1000) {
	 if (sensorValues[1] - sensorminvalue[1] < 1000) //line is out of sight from the central sensor
	 {
	 if (lineposition < 300 && lineposition > -300) {
	 lineposition_state = POSITION_BAD; //completely out of sight
	 lineposition = lastlineposition;
	 } else if (lineposition_state != POSITION_BAD) { //cannot go from bad to critical
	 lineposition_state = POSITION_CRITICAL; //line is on the outer edge of field of view
	 if (lineposition > 0)
	 lineposition = SENSORSPACING / 2;
	 else
	 lineposition = -SENSORSPACING / 2;

	 lastlineposition = lineposition;
	 }
	 }
	 } else //if position error is big, the line is in sight
	 {
	 lineposition_state = POSITION_OK;
	 lastlineposition = lineposition;
	 }*/
	//if sum of all sensor values is small, we lost sight of the line
	if (sensorValues[0] + sensorValues[1] + sensorValues[2] < 1400) {
		lineposition_state = POSITION_BAD; //completely out of sight
		if(lastlineposition>0)lineposition = SENSORSPACING / 2;
		else lineposition = -SENSORSPACING / 2;

	} else //if at least one sensor shows a high value, there is a line
	{
		lineposition_state = POSITION_OK;
		lastlineposition = lineposition;
	}
	return lineposition;
}

//PID regulator:
//the regulator tries to get the line to the zero position by changing the
//speeds of the motors proportionally to the offset error (P-regulator)
//the added integral of the error removes the offset any P-regulator has
//the differential part acts as a 'damping' effect for the integral part and also does a 'look ahead' (by looking at change of error rather than the error itself)
//the ideal parameters are found by experimentation. The code must set some limits to the PID parameters (kp, ki, kd) but the parameters themselves are set by the potentiometers

//input to the regulator is the error of the position, output is the amount of 'steering' needed

int32_t PIDregulator(int16_t error) {
	static int16_t lasterror; //to calculate the differential
	static int16_t integratederror = 0; //the calculated integral
	static int16_t differential;
//	static int32_t differentiallowpass = 0;
	int32_t output;

	volatile int32_t P;
	volatile int32_t I;
	volatile int32_t D;

	//calculate the integral
	integratederror += error;
	if (integratederror > ANTIWINDUP_MAX)
		integratederror = ANTIWINDUP_MAX;
	if (integratederror < -ANTIWINDUP_MAX)
		integratederror = -ANTIWINDUP_MAX;
	/*
	 //reset the integrated error when crossing the center
	 if ((lasterror > 0 && error < 0) || (lasterror < 0 && error > 0))
	 integratederror = 0;
	 */
	if (lineposition_state == POSITION_OK) {
		integratederror = 0;
	}

	/*
	 //bring the integrated error back to zero over time (damping)
	 if (integratederror > 7)
	 integratederror -= 15;
	 else if (integratederror < -7)
	 integratederror += 15;
	 */
	//calculate the differential
	differential = (differential + (lasterror - error)) / 2;
	//use a lowpass to even out spikes in readings: output = (x * last_output + present_reading) / (x+1)
	//differentiallowpass = ((7*differentiallowpass)+(int32_t (lasterror - error)))>>3;

	lasterror = error;

	//PID regulator with cubed proportional output
	//P = (int32_t) potentiometers[0] * error* error * error;
	//if(error < 0) P = -P;
	//I = (int32_t) (potentiometers[1]<<5) * integratederror;
	//D = (int32_t) (potentiometers[2]<<2) * differential;
	P = (int32_t) (potentiometers[0]>>1) * error;
	//if(error < 0) P = -P;
	I = (int32_t) (potentiometers[1] >> 2) * integratederror;
	D = (int32_t) (potentiometers[2] >> 2) * differential;
	//D = 0;

	//I=0;
	output = P + I + D;

	return output;
}

//robot control function to be called periodically at a fixed interval
void RobotControl(void) {
	volatile int16_t positionerror;
	volatile int32_t regulatoroutput;
	static int32_t lastregulatoroutput;
	speed = (potentiometers[3] >> 5); //speed as pwm setting of 0-255 from 12bit ADC value, divide by two to make regulator symmetrical (speed/2±regulator)

	if (speed > MAXSPEED / 2)
		speed = MAXSPEED / 2;

	//read the sensor values:
	readSensorValues();

	//detect line crossing (not working right):
	//if (sensorValues[0] + sensorValues[1] + sensorValues[2] > 11000) {
//		positionerror = SENSORSPACING / 2;
//	} else {
	//calculate the line position (= offset in micrometers from the center)
	positionerror = calculateLinePosition() >> 4; //divide by 16 to not make the PID values overflow
//	}

	//get the regulator output:
	regulatoroutput = PIDregulator(positionerror);

	//the maximum output from the regulator is SENSORSPACING*4096 + ANTIWINDUP_MAX*4096 + DIFFMAX*4096
	//where DIFFMAX is the maximum differential error between two calls, let's assume it is SENSORSPACING/2
	//this totals in 54'272'000. This value must be divided down to give the maximum PWM differce between the two motors
	//the default PWM value for regulatoroutput = 0 is equal to 'speed'.
	//the maximum PWM output is 255 (it is scaled to the timer period in the set pwm function)
	//to regulate the direction one of the motors speed is decreased.
	//left motor is slowed down when the regulator output is smaller than zero
	//right motor is slowed down when the regulator output is bigger than zero

	//the right motor (Motor A) is connected to PA6 or TIM3 channel 1
	//the left motor (Motor B) is connected to PA7 or TIM3 channel 2

	//scale down the regulatoroutput (value found by experimentation):
	//regulatoroutput = regulatoroutput >> 23; //use this for cubed output
	//regulatoroutput = regulatoroutput >> 15; //use this for squared output
	//regulatoroutput = regulatoroutput >> 12; //use this for linear output
	regulatoroutput = regulatoroutput >> 13; //use this for linear output

	//limit pwm regulation according to the maximum speed
	//if(regulatoroutput < 50) speed-= 5; //slow down a little
	if (regulatoroutput > speed)
		regulatoroutput = speed;
	if (regulatoroutput < -speed)
		regulatoroutput = -speed;

	//scale the speed according to input voltage (updated in main loop)
	//the reference is 3.9V, above: scale down, below: scale up

	//check if line detected
	if (lineposition_state == POSITION_BAD) //line has gone out of sight
			{
		//regulatoroutput = lastregulatoroutput;
		LED_OFF;
	} else //line is in sight
	{
		lastregulatoroutput = regulatoroutput;
		LED_ON;
	}

	speed = ((int32_t) speed * 3900) / ((int32_t) Vbat); //4000mV as reference

	PWM_SetDuty(1, speed - regulatoroutput); // speed on motor A
	PWM_SetDuty(2, speed + regulatoroutput); // speed on motor B

	/*
	 if (regulatoroutput >= 0) //line is on the right side, slow down motor A
	 {
	 PWM_SetDuty(1, speed - regulatoroutput * regulatoroutput); //reduced speed on motor A
	 PWM_SetDuty(2, speed + (regulatoroutput * regulatoroutput) / 2); //full speed on motor B
	 } else //line is on the left side, slow down motor B
	 {
	 PWM_SetDuty(1, speed + (regulatoroutput * regulatoroutput) / 2); //reduce speed on motor A
	 PWM_SetDuty(2, speed - (regulatoroutput * regulatoroutput)); //reduced speed on motor B (regulatoroutput is negative)
	 }*/
}

