/* 
* RotaryEncoder.cpp
*
* Created: 23.04.2016 23:13:20
* Author: JochenAlt
*/

#include "Arduino.h"
#include "RotaryEncoder.h"

// default constructor
RotaryEncoder::RotaryEncoder(int number)
{
	sensor.setI2CAddress(AS5048_ADDRESS+number);
} //RotaryEncode

void RotaryEncoder::setNullPosition() {
	//consider the current position as zero
	sensor.setZeroReg();
}

float RotaryEncoder::getAngle() {
	return sensor.angleR(U_DEG, true);
}
