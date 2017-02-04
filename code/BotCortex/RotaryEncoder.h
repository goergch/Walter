/* 
* RotaryEncoder.h
*
* Interface to AMS 5048B encoders with 14bit angle resolution connected via I2C
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

	// set or return null value
	float getNullAngle();
	void setNullAngle(float angle);

	// get null-corrected angle
	float getAngle();

	// forearm has an offset of 90°, the other actuators have offset of 0°
	float getAngleOffset();

	// read most recent angle that has been fetched (do not contact sensor, value is cached)
	float getRawSensorAngle();

	// fetch new angle from sensor
	bool readNewAngleFromSensor();

	// fetch a couple of samples and compute variance (used to check if sensor works ok)
	bool fetchSample(float& avr, float& variance);

	// check if encoders works correctly (by use fo fetchSample)
	float checkEncoderVariance();

	// is sensor up and running correctly?
	bool isOk() {
		return isCommunicationOk() && isCheckOk() && isFailedReadingOk();
	}

	bool isCommunicationOk() {
		return communicationWorks;
	}

	bool isCheckOk() {
		return passedCheck;
	}

	bool isFailedReadingOk() {
		return (failedReadingCounter < 5);
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
	bool filterAngle = true;
}; //RotaryEncode

#endif //__ROTARYENCODE_H__
