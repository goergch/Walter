/* 
* MotorDriver.h
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#ifndef __MOTORDRIVER_HERKULEX_IMPL_H__
#define __MOTORDRIVER_HERKULEX_IMPL_H__


#include <Arduino.h>
#include "Actuator.h"
#include "ActuatorConfig.h"
#include "HerkuleX.h"

class HerkulexServoDrive: public DriveBase
{
//functions
public:
	HerkulexServoDrive(): DriveBase (){
		configData = NULL;
		setupData = NULL;
		beforeFirstMove = true;
		currentAngle = 0;
		overloadDetected = false;
		anyHerkulexError = false;
		torque = 0.0;
		lastAngle = 0;
	}
	void setAngle(float angle,uint32_t pDuration_ms);
	void changeAngle(float pAngleChange,uint32_t pAngleTargetDuration);
	
	void setup( ServoConfig* config, ServoSetupData* setupData);
	void loop(uint32_t now);
	float getCurrentAngle();
	float getRawAngle();
	
	void setNullAngle(float pAngle);
	void readFeedback(float &angle, float &torque, bool& overLoad, bool &anyerror);
	bool isOk();
	
	ServoConfig& getConfig() { return *configData;}

private:	
	void moveToAngle(float angle, uint32_t pDuration_ms);
	bool beforeFirstMove;

	float currentAngle;
	boolean overloadDetected;	// used to store servo feedback, true of too much load on the servo 
	float torque;				// trial to compute current torque out of required pwm value 
	boolean anyHerkulexError;
	ServoConfig* configData;
	ServoSetupData* setupData;
	float lastAngle;
}; //MotorDriver

#endif //__MOTORDRIVER_HERKULEX_IMPL_H__
