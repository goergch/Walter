/* 
* RotaryEncoder.h
*
* Created: 23.04.2016 23:13:21
* Author: JochenAlt
*/


#ifndef __ROTARYENCODE_H__
#define __ROTARYENCODE_H__

#include "Arduino.h"
#include "setup.h"
#include "AMS_AS5048B.h"

class RotaryEncoder
{
public:
	RotaryEncoder() { myNumber = 0;zeroPosition = 0;currentSensorAngle = 0;};
	void setup(uint8_t number);
	void setNullPosition();
	float getAngle();
	void fetchAngle();
	static void switchConflictingSensor(bool powerOn);

private:
	bool isClockwise() {
		return EncoderConfig[myNumber-1].clockwise;
	}
	uint8_t i2CAddress(bool after) {
		if (after)
			return EncoderConfig[myNumber-1].I2CAddress + I2C_ADDRESS_ADDON;
		else
			return EncoderConfig[myNumber-1].I2CAddress;
	}
	
	bool reprogrammei2CAddress() {
		return EncoderConfig[myNumber-1].programmI2CAddress;
	}

	AMS_AS5048B sensor;
	uint8_t myNumber;
	float currentSensorAngle;
	float zeroPosition;
}; //RotaryEncode

#endif //__ROTARYENCODE_H__
