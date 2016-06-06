/* 
* Motors.h
*
* Created: 22.04.2016 19:26:44
* Author: JochenAlt
*/


#ifndef __MOTORS_H__
#define __MOTORS_H__


#include "Arduino.h"
#include "Actuator.h"
#include "HerkulexServoDrive.h"
#include "GearedStepperDrive.h"
#include "RotaryEncoder.h"

#include "TimePassedBy.h"

class Controller {
	public:
		Controller();
		Controller( const Controller&c );


		void printMenuHelp();
		void interactiveLoop();
		void interactive(bool on);
		bool interactive() { return interactiveOn;}

		void setup();
		bool checkEncoders();
		void printStepperConfiguration();
		void loop();
		void stepperLoop();
		void printEncoderAngles();

	private:
		void transferEncoderAngle(int motorNumber);
		Actuator* getMotor(int motorNumber);	
		HerkulexServoDrive wristMotor;
		GearedStepperDrive stepper[MAX_MOTORS-1];
		RotaryEncoder	encoders[MAX_ENCODERS];
		uint8_t numberOfMotors;
		uint8_t numberOfEncoders;
		uint8_t numberOfSteppers;

		Actuator* currentMotor;				// currently set motor used for interaction
		TimePassedBy motorKnobTimer;			// used for measuring sample rate of motor knob
		bool interactiveOn;
}; //Motors

#endif //__MOTORS_H__
