/*
* Actuator.h
*
* Common base class of servos and stepper motors. This is used by
* the trajectory execution to move any actuator regardless of its type
*
* Author: JochenAlt
*/


#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__


#include <Arduino.h>
#include "RotaryEncoder.h"
#include "Config.h"
#include "GearedStepperDrive.h"
#include "HerkulexServoDrive.h"

#include "DriveBase.h"
class Actuator
{
	public:
		Actuator();
		Actuator( const Actuator &c );
	
		bool isInitialized() { return hasBeenInitialized;}
		void setup(ActuatorConfig* pConfigData, HerkulexServoDrive* servo);
		void setup(ActuatorConfig* pConfigData, GearedStepperDrive* stepper, RotaryEncoder* encoder);

		void setup();
 			
		void loop(uint32_t now) {
			drive()->loop(now);
		}
		void setAngle(float angle,uint32_t pDuration_ms) {
			drive()->setAngle(angle,pDuration_ms);
		};
		void changeAngle(float angle,uint32_t pDuration_ms) {
			drive()->changeAngle(angle,pDuration_ms);
		};
		float getCurrentAngle() {
			return drive()->getCurrentAngle();
		}
		void enable() {
			return drive()->enable();
		}
		void disable() {
			return drive()->disable();
		}

		bool setCurrentAsNullPosition();

		void printConfiguration();

		inline bool hasEncoder() { return encoder != NULL; }
		inline bool hasStepper() { return stepperDrive != NULL; }
		inline bool hasServo() { return servoDrive != NULL; }

		RotaryEncoder& getEncoder () { return *encoder; }
		GearedStepperDrive& getStepper() { return *stepperDrive; }
		HerkulexServoDrive& getServo() { return *servoDrive; }
			
		void setMaxAngle(float angle);
		void setMinAngle(float angle);
		void setNullAngle(float angle);

		void setD(float D);
		void setP(float P);
		void setI(float I);

		void setMaxSpeed(float maxSpeed);
		void setMaxAcc(float maxAcc);

		float getMaxAngle();
		float getMinAngle();
		float getNullAngle();
		
		ActuatorConfig& getConfig() { return *configData; };
		void printName();

	protected:
		DriveBase* drive() { return (stepperDrive != NULL)?(DriveBase*)stepperDrive:(DriveBase*)servoDrive;};
		ActuatorConfig* configData;

		RotaryEncoder* encoder;
		GearedStepperDrive* stepperDrive;
		HerkulexServoDrive* servoDrive;
	private:
		bool hasBeenInitialized;
}; // Actuator



#endif //__MOTORDRIVER_H__
