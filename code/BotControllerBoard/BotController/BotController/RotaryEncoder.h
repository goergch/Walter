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
#include "ActuatorConfig.h"

class RotaryEncoder
{
public:
	RotaryEncoder() { 
		currentSensorAngle = 0;
		passedCheck = false;
		configData = NULL;
		setupData=NULL;
	};
	void setup( RotaryEncoderConfig& config, RotaryEncoderSetupData& setupData);
	RotaryEncoderConfig& getConfig() { return *configData;};
	void setNullAngle(float angle);
	float getNullAngle();

	float getAngle();
	float getRawSensorAngle();

	void fetchAngle();
	bool fetchSample(bool raw,float& avr, float& variance);

	float checkEncoderVariance();
	bool isOk() {
		return passedCheck;
	}
	static void switchConflictingSensor(bool powerOn);

private:
	bool fetchSample(bool raw,uint8_t no, float sample[], float& avr, float& variance);
	bool isClockwise() {return setupData->clockwise;}
	uint8_t i2CAddress(bool after) {
		if (after)
			return setupData->I2CAddress + I2C_ADDRESS_ADDON;
		else
			return setupData->I2CAddress;
	}
	
	bool reprogrammei2CAddress() {
		return setupData->programmI2CAddress;
	}

	bool isReverse() {
		return setupData->reverse;
	}
	AMS_AS5048B sensor;
	float currentSensorAngle;
	RotaryEncoderSetupData* setupData;
	RotaryEncoderConfig* configData;
	bool passedCheck;

}; //RotaryEncode

#endif //__ROTARYENCODE_H__
