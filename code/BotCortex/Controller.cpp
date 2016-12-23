/* 
* Motors.cpp
*
* Created: 22.04.2016 19:26:44
* Author: JochenAlt
*/

#include "Arduino.h"
#include "utilities.h"
#include "BotMemory.h"
#include "Controller.h"
#include <avr/wdt.h>
#include <I2CPortScanner.h>
#include "GearedStepperDrive.h"
#include "RotaryEncoder.h"
#include "watchdog.h"


Controller controller;
TimePassedBy servoLoopTimer;
TimePassedBy encoderLoopTimer;
uint8_t adjustWhat = ADJUST_MOTOR_MANUALLY;

// stepperloop needs to be called as often as possible, since the stepper impulse has to happen every 200us at top speed
// So, call that whereever you can, even during delay() 
void plainStepperLoop() {
	controller.stepperLoop(); // take care that nothing happens inside until everything is properly initialized
}

// yield is called in delay(), mainly used to leverage serial communication time 
void yield() {
	controller.stepperLoop();
}

Controller::Controller()
{
	currentMotor = NULL;				// currently set motor used for interaction
	numberOfActuators = 0;				// number of motors that have been initialized
	numberOfEncoders = 0;				// number of rotary encoders that have been initialized
	numberOfSteppers = 0;				// number of steppers that have been initialized
	setuped= false;					// flag to indicate a finished setup (used in stepperloop())
	enabled = false;					// disabled until explicitly enabled
}

void Controller::enable() {
	if (isSetup()) {
		for (int i = 0;i<numberOfActuators;i++) { 
			getActuator(i)->enable();
			// give it a break to not overload power supply by switching on all steppers at the same time
			delay(5);
		}
		enabled = true;
	}
}

void Controller::disable() {
	if (isSetup()) {
		for (int i = 0;i<numberOfActuators;i++) {
			getActuator(i)->disable();
			// give it a break to not overload power supply by switching off all steppers at the same time
			delay(10);
		}
		enabled = false;
	}
}

void Controller::selectActuator(uint8_t no) {
	// disable old motor
	if (currentMotor != NULL) {
		currentMotor->disable();
		currentMotor = NULL;
	}
	
	if ((no>=0) || (no<MAX_ACTUATORS)) {
		currentMotor = getActuator(no);
		if (currentMotor != NULL)
			currentMotor->enable();
	} 
}

Actuator* Controller::getCurrentActuator() {
	return currentMotor;
}

void Controller::printConfiguration() {
	logger->println(F("ACTUATOR SETUP"));
	for (int i = 0;i<numberOfActuators;i++) {
		// ActuatorSetupData* thisActuatorSetup = &actuatorSetup[i];
		// ActuatorIdentifier id = thisActuatorSetup->id;
		//thisActuatorSetup->print();
		for (int j = 0;j<numberOfServos;j++) {
			ServoSetupData* thisServoSetup = &servoSetup[j];
			if (thisServoSetup->id == i) {
				logger->print(F("   "));
				thisServoSetup->print();
			}			
		}
		for (int j = 0;j<numberOfSteppers;j++) {
			StepperSetupData* thisStepperSetup = &stepperSetup[j];
			if (thisStepperSetup->id == i) {
				logger->print(F("   "));
				thisStepperSetup->print();
			}
		}
		for (int j = 0;j<numberOfEncoders;j++) {
			RotaryEncoderSetupData* thisEncoderSetup = &encoderSetup[j];
			if (thisEncoderSetup->id == i) {
				logger->print(F("   "));
				thisEncoderSetup->print();
			}
		}

	}
	
	logger->println(F("ACTUATOR CONFIG"));
	memory.println();
}

bool Controller::setup() {
	watchdogReset(); // this takes a bit longer, kick the dog regularly

	bool result = true;;
	numberOfActuators = 0;
	numberOfSteppers = 0;
	numberOfEncoders = 0;
	numberOfServos = 0;

	// setup requires power for Herkulex servos
	switchServoPowerSupply(true);
	
	if (memory.persMem.logSetup) {
		logger->println(F("--- com to I2C bus"));
		logger->print(F("    "));
		int devices0 = doI2CPortScan(F("I2C0"),Wires[0], logger);
		logger->print(F("    "));
		logger->print(devices0);
		if (devices0 != 4)
			logger->println(F(" devices on I2C0 found, 4 expected"));
		else
			logger->println(F(" devices found, ok"));
		logger->print(F("    "));
		int devices1 = doI2CPortScan(F("I2C1"),Wires[1], logger);
		logger->print(F("    "));
		logger->print(devices1);

		if (devices1 != 1)
			logger->println(F(" devices on I2C1 found, 1 expected"));
		else
			logger->println(F(" devices found, ok"));

	}

	if (memory.persMem.logSetup) {
		logger->println(F("--- com to servo"));
	}
	
	watchdogReset(); // this takes a bit longer, kick the dog regularly
	HerkulexServoDrive::setupCommunication();

	if (memory.persMem.logSetup) {
		logger->println(F("--- initializing actuators"));
	}

	for (numberOfActuators = 0;numberOfActuators<MAX_ACTUATORS;numberOfActuators++) {
		watchdogReset(); // this takes a bit longer, kick the dog regularly

		if (memory.persMem.logSetup) {
			logger->print(F("--- setup "));
			logActuator((ActuatorIdentifier)numberOfActuators);
			logger->println(F(" ---"));
		}

		Actuator* thisActuator = &actuators[numberOfActuators];
		ActuatorConfig* thisActuatorConfig = &(memory.persMem.armConfig[numberOfActuators]);
		switch (thisActuatorConfig->actuatorType) {
			case SERVO_TYPE: {
				if (numberOfServos >= MAX_SERVOS)
					logFatal(F("too many servos"));

				HerkulexServoDrive* servo = &servos[numberOfServos];
				bool ok  = servo->setup( &(memory.persMem.armConfig[numberOfActuators].config.servoArm.servo), &(servoSetup[numberOfServos])); 
				if (!ok)
					result = false;
				thisActuator->setup(thisActuatorConfig, servo);
				numberOfServos++;

				if (thisActuator->hasStepper())
					logFatal(F("misconfig: stepper!"));
				if (thisActuator->hasEncoder())
					logFatal(F("misconfig: encoder!"));
				if (!thisActuator->hasServo())
					logFatal(F("misconfig: no servo"));

				break;
			}
			/*
			case STEPPER_ENCODER_TYPE: {
				if (numberOfEncoders >= MAX_ENCODERS)
					logFatal(F("too many encoders"));
				if (numberOfSteppers >= MAX_STEPPERS)
					logFatal(F("too many steppers"));

				RotaryEncoder* encoder = &encoders[numberOfEncoders];
				GearedStepperDrive* stepper = &steppers[numberOfSteppers];

				encoder->setup(&actuatorConfigType[numberOfActuators], &(thisActuatorConfig->config.stepperArm.encoder), &(encoderSetup[numberOfEncoders]));
				stepper->setup(&(thisActuatorConfig->config.stepperArm.stepper), &actuatorConfigType[numberOfActuators], &(stepperSetup[numberOfSteppers]));
				thisActuator->setup(thisActuatorConfig, stepper, encoder);

				if (!thisActuator->hasStepper()) 
					logFatal(F("misconfig: no stepper"));
				if (!thisActuator->hasEncoder())
					logFatal(F("misconfig: no encoder"));
				if (thisActuator->hasServo())
					logFatal(F("misconfig: servo!"));

				numberOfEncoders++;
				numberOfSteppers++;
				break;
			}
			*/
			default:
				logFatal(F("unknown actuator type"));
		}
	}
	
	if (memory.persMem.logSetup) {
		logger->println(F("--- check encoders"));
	}
	
	// get measurement of encoder and ensure that it is plausible 
	// (variance of a couple of samples needs to be low)
	result = true;
	for (int i = 0;i<numberOfEncoders;i++) {
		bool encoderCheckOk = checkEncoder(i);
		logger->print(F("enc(0x"));
		logger->print(encoders[i].i2CAddress(), HEX);
		logger->print(F(")"));
		if (!encoderCheckOk) {
			logger->println(F(" not ok!"));
			result = false;
		} else {
			logger->println(F(" ok"));
		}
	}
	
	// set measured angle of the actuators and define that angle as current position by setting the movement
	for (int i = 0;i<numberOfActuators;i++) {
		Actuator* actuator = getActuator(i);
		if (actuator->hasEncoder()) {

			RotaryEncoder& encoder = actuator->getEncoder();

			// find corresponding stepper
			if (actuator->hasStepper()) {
				GearedStepperDrive& stepper= actuator->getStepper();
				if (encoder.getConfig().id != stepper.getConfig().id) {
					logActuator(stepper.getConfig().id);
					logFatal(F("encoder and stepper different"));
					result = false;
				} else {
					if (encoder.isOk()) {
						float angle = encoder.getAngle();
						stepper.setCurrentAngle(angle);  // initialize current motor angle
						stepper.setMeasuredAngle(angle, millis()); // tell stepper that this is a measured position		
						stepper.setAngle(angle,1);	     // define a current movement that ends up at current angle. Prevents uncontrolled startup.
					}
					else  {
						logActuator(stepper.getConfig().id);
						logFatal(F("encoder not ok"));
						result = false;
					}
				}
			} else {
					actuator->printName();
					logFatal(F("encoder has no stepper"));				
					result = false;
			}
		}
	}
	
	if (memory.persMem.logSetup) {
		logger->println(F("--- initialize ADC"));
	}
	
	// knob control of a motor uses a poti that is measured with the internal adc
	analogReference(DEFAULT); // use voltage of 3.3V as reference
	
	setuped= true;
	if (memory.persMem.logSetup) {
		logger->println(F("setup done"));
	}

	// if setup is not successful power down servos
	if (!result)
		switchServoPowerSupply(false);
		
	return result;
}

Actuator* Controller::getActuator(uint8_t actuatorNumber) {
	if ((actuatorNumber>= 0) && (actuatorNumber<numberOfActuators))
		return &actuators[actuatorNumber];
	else 
		return NULL;
}	

void Controller::adjustMotor(int adjustmentType) {
	adjustWhat = adjustmentType;
}

void Controller::changeAngle(float incr, int duration_ms) {
	if (currentMotor != NULL) {
		currentMotor->changeAngle(incr, duration_ms);
	}
}

void Controller::switchActuatorPowerSupply(bool on) {
	if (on) {
		digitalWrite(POWER_SUPPLY_STEPPER_PIN, HIGH);	// start with stepper to not confuse servo by impulse
		delay(100);
	} else {
		digitalWrite(POWER_SUPPLY_STEPPER_PIN, LOW);	// start with stepper to not confuse servo by impulse
		digitalWrite(POWER_SUPPLY_SERVO_PIN, LOW);		// switch off servo too
	}
	powered = on;
}

void Controller::switchServoPowerSupply(bool on) {
	if (on) {
		digitalWrite(POWER_SUPPLY_SERVO_PIN, HIGH);	 // switch relay to give power to Herkulex servos
		delay(50); // herkulex servos need that time before receiving commands
	} else {
		digitalWrite(POWER_SUPPLY_SERVO_PIN, LOW);		
	}
}


void Controller::stepperLoop() {
	if (isSetup()) {
		// call all stepper loops as often as you can, this makes the timing of the movement precise and smooth
		for (uint8_t i = 0;i< numberOfSteppers ;i++) {
			steppers[i].loop();
		}
	}
}

void Controller::loop(uint32_t now) {

	stepperLoop(); // send impulses to steppers

	// loop that checks the proportional knob	
	if (currentMotor != NULL) {
		if (adjustWhat == ADJUST_MOTOR_BY_KNOB) {
			if (motorKnobTimer.isDue_ms(MOTOR_KNOB_SAMPLE_RATE, now)) {
				// fetch value of potentiometer, returns 0..1024 representing 0..2.56V
				int16_t adcValue = analogRead(MOTOR_KNOB_PIN);

				// compute angle out of adcDiff, potentiometer turns between 0°..270°
				float angle = (float(adcValue-512)/512.0) * (270.0 / 2.0);			
				static float lastAngle = 0;

				if ( (abs(adcValue-512)<500) &&  (lastAngle != 0)) {
					// if the sensor is active, set an absolute angle, otherwise, use a relative one
					if (getCurrentActuator()->hasServo() || 
						(getCurrentActuator()->hasEncoder() && getCurrentActuator()->getEncoder().isOk())) {
						logger->print(F("knob:set to "));
						logger->println(angle,1);
						currentMotor->setAngle(angle,MOTOR_KNOB_SAMPLE_RATE);
					}
					else {
						logger->print(F("knob:adjust by "));
						logger->println(angle-lastAngle,1);
						currentMotor->changeAngle(angle-lastAngle,MOTOR_KNOB_SAMPLE_RATE);
					}
				}
				
				
				lastAngle = angle;				
			}
		}
	};
	
	// update the servos
	// with each loop just one servo (time is rare due to steppers)
	if (servoLoopTimer.isDue_ms(SERVO_SAMPLE_RATE,now)) {
		for (int i = 0;i<MAX_SERVOS;i++) {
			servos[i].loop(now);
			stepperLoop(); // send impulses to steppers
		}
	}

	// fetch the angles from the encoders and tell the stepper controller
	if (encoderLoopTimer.isDue_ms(ENCODER_SAMPLE_RATE, now)) {
		// fetch encoder values and tell the stepper measure 
		// logger->println();
		for (int encoderIdx = 0;encoderIdx<numberOfEncoders;encoderIdx++) {
			// find corresponding actuator
			ActuatorIdentifier actuatorID = encoders[encoderIdx].getConfig().id;
			Actuator* actuator = getActuator(actuatorID);
			if (actuator->hasStepper()) {
				GearedStepperDrive& stepper = actuator->getStepper();
				if (stepper.getConfig().id != actuatorID) {
					logActuator(actuatorID);
					logger->print(actuatorID);
					logger->print(encoderIdx);
					logger->print(stepper.getConfig().id);
					logFatal(F("wrong stepper identified"));
				}
				float currentAngle = stepper.getCurrentAngle();

				if (encoders[encoderIdx].isOk()) {
					stepperLoop(); // send impulses to steppers

					bool commOk = encoders[encoderIdx].getNewAngleFromSensor(); // measure the encoder's angle
					stepperLoop(); // send impulses to steppers

					if (commOk) {						
						currentAngle = encoders[encoderIdx].getAngle();
					}
				} 
				stepper.setMeasuredAngle(currentAngle, millis());	
				stepperLoop();
			}
		}
	}		

	if (memory.persMem.logEncoder) 
		printAngles();
}

void Controller::printAngles() {
	logger->print(F("angles{"));
	for (int actNo = 0;actNo<numberOfActuators;actNo++) {
		Actuator* actuator=  getActuator(actNo);
		logActuator(actuator->getConfig().id);
		
		if (actuator->hasEncoder()) {
			logger->print(F(" enc="));

			RotaryEncoder& encoder = actuator->getEncoder();
			float measuredAngle = encoder.getAngle();
			logger->print(measuredAngle,2);
			logger->print("(");
			measuredAngle = encoder.getRawSensorAngle();
			logger->print(measuredAngle,2);
			logger->print(")");

		}
		
		if (actuator->hasServo()) {
			logger->print(F(" srv="));
			HerkulexServoDrive& servo= actuator->getServo();
			float measuredAngle = servo.getCurrentAngle();
			logger->print(measuredAngle);
			logger->print("(");
			
			measuredAngle = servo.getRawAngle();
			logger->print(measuredAngle);
			logger->print(") ");
		}
	}
		
		
	logger->println("}");
}

bool  Controller::checkEncoder(int encoderNo) {
	bool ok = true;
	wdt_reset(); // this might take longer
	encoders[encoderNo].checkEncoderVariance(); 
	if (!encoders[encoderNo].isOk())			// isOk returns if communication and checkEncoderVariance went fine
		ok = false;

	return ok;
}
