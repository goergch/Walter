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
#include <HkxPosControl.h>

class HerkulexServoDrive: public Actuator
{
//functions
public:
	HerkulexServoDrive(): Actuator (){
		servo = NULL;
		mostRecentAngle = 0;
		firstMove = true;
	}
	void setAngle(float angle,uint32_t pDuration_ms);
	void changeAngle(float pAngleChange,uint32_t pAngleTargetDuration);

		
	
	void setup(int motorNumber);
	virtual void loop(uint32_t now);
	virtual void moveToAngle(float angle, uint32_t pDuration_ms);
	virtual float getCurrentAngle();
	float readCurrentAngleFromServo();
	
private:	
	HkxPosControl* servo;
	float mostRecentAngle;
	bool firstMove;
}; //MotorDriver

#endif //__MOTORDRIVER_HERKULEX_IMPL_H__
