/* 
* MotorDriver.h
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#ifndef __MOTORDRIVER_HERKULEX_IMPL_H__
#define __MOTORDRIVER_HERKULEX_IMPL_H__


#include <Arduino.h>
#include "MotorDriver.h"
#include <HkxPosControl.h>

class MotorDriverHerkulexImpl: public MotorDriver
{
//functions
public:
	MotorDriverHerkulexImpl(): MotorDriver (){
		servo = NULL;
		currentAngle = 0;
	}
	
	void setup(int motorNumber);
	virtual void loop();
	virtual void setRawAngle(float angle, uint32_t pDuration_ms);
	virtual float getRawAngle();
		
private:	
	void updateCurrentAngle();
	HkxPosControl* servo;
	float currentAngle;
	float lastAngleSetting;
}; //MotorDriver

#endif //__MOTORDRIVER_HERKULEX_IMPL_H__
