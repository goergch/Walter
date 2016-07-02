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

		void printMenuHelp();
		void interactiveLoop();
		void interactive(bool on);
		bool interactive() { return interactiveOn;}

		void setup();
		bool checkEncoders();
		void printSetupConfiguration();
		void loop();
		void stepperLoop();
		void printAngles();
		Actuator* getActuator(uint8_t number);
		bool setupIsDone() { return setupDone;};
		void enable();
		void disable();

		bool isEnabled() { return enabled;}
	private:
		HerkulexServoDrive	servos[MAX_SERVOS];
		GearedStepperDrive	steppers[MAX_STEPPERS];
		RotaryEncoder		encoders[MAX_ENCODERS];
		Actuator			actuators[MAX_ACTUATORS];

		uint8_t numberOfActuators;
		uint8_t numberOfEncoders;
		uint8_t numberOfSteppers;
		uint8_t numberOfServos;

		Actuator* currentMotor;				// currently set motor used for interaction
		TimePassedBy motorKnobTimer;		// used for measuring sample rate of motor knob
		bool interactiveOn;
		bool setupDone;
		bool enabled;
}; //Motors

#endif //__MOTORS_H__
