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
	RotaryEncoder() { myNumber = 0;currentSensorAngle = 0;passedCheck = false;};
	void setup(uint8_t number);
	void setNullAngle(float angle);
	float getNullAngle();

	float getAngle();
	float getRawAngle();

	void fetchAngle();
	static void switchConflictingSensor(bool powerOn);
	bool fetchSample(bool raw,float& avr, float& variance);

	float checkEncoderVariance();
	bool isOk() {
		return passedCheck;
	}

private:
	bool fetchSample(bool raw,uint8_t no, float sample[], float& avr, float& variance);

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

	bool isReverse() {
		return EncoderConfig[myNumber-1].reverse;
	}
	AMS_AS5048B sensor;
	uint8_t myNumber;
	float currentSensorAngle;
	bool passedCheck;
}; //RotaryEncode

#endif //__ROTARYENCODE_H__
