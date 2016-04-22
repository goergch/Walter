/* 
* Motors.h
*
* Created: 22.04.2016 19:26:44
* Author: JochenAlt
*/


#ifndef __MOTORS_H__
#define __MOTORS_H__


#include "Arduino.h"
#include "MotorDriver.h"
#include "MotorDriverherkulexImpl.h"
#include "TimePassedBy.h"

class Motors {
	public:
		Motors();
		Motors( const Motors&c );


		void printMenuHelp();
		void interactiveLoop();
	
		void setup();
		void loop();
	private:
		MotorDriver* getMotor(int motorNumber);	

		MotorDriverHerkulexImpl wristMotor;
		static MotorDriver* motorDriverArray[MAX_MOTORS];
		uint8_t numberOfMotors;
		MotorDriver* currentMotor;				// currently set motor used for interaction
		bool motorKnobOn;						// true if motorknob mode is on
		TimePassedBy motorKnobTimer;			// used for measuring sample rate of motor knob
}; //Motors

#endif //__MOTORS_H__
