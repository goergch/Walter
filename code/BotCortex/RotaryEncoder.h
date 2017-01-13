/* 
* RotaryEncoder.h
*
* Created: 23.04.2016 23:13:21
* Author: JochenAlt
*/


#ifndef __ROTARYENCODE_H__
#define __ROTARYENCODE_H__

#include "Arduino.h"
#include "AMS_AS5048B.h"
#include "Config.h"
#include "ActuatorProperty.h"
#include "pins.h"

class RotaryEncoder
{
public:
	RotaryEncoder() { 
		currentSensorAngle = 0;
		passedCheck = false;
		configData = NULL;
		setupData=NULL;
		communicationWorks = false;
		failedReadingCounter = 0;
	};
	void setup( ActuatorConfiguration* pActuatorConfig, RotaryEncoderConfig* config, RotaryEncoderSetupData* setupData);
	RotaryEncoderConfig& getConfig() { return *configData;};
	void setNullAngle(float angle);
	float getNullAngle();

	float getAngle();
	float getAngleOffset();

	float getRawSensorAngle();

	bool getNewAngleFromSensor();
	bool fetchSample(float& avr, float& variance);

	float checkEncoderVariance();
	bool isOk() {
		return communicationWorks & passedCheck & (failedReadingCounter < 8);
	}
	uint8_t i2CAddress() {	return setupData->I2CAddress;}
	i2c_t3* i2CBus() {	return Wires[setupData->I2CBusNo];}

private:
	bool fetchSample(uint8_t no, float sample[], float& avr, float& variance);
	bool isClockwise() {return setupData->clockwise;}

	AMS_AS5048B sensor;
	float currentSensorAngle;
	RotaryEncoderSetupData* setupData;
	RotaryEncoderConfig* configData;
	ActuatorConfiguration* actuatorConfig;
	bool passedCheck;
	bool communicationWorks;
	uint8_t failedReadingCounter;
	uint32_t lastSensorRead = 0;
}; //RotaryEncode

#endif //__ROTARYENCODE_H__
