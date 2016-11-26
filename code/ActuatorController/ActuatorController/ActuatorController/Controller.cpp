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
#include "GearedStepperDrive.h"
#include "RotaryEncoder.h"



extern Controller controller;
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
	plainStepperLoop();	
}

Controller::Controller()
{
	currentMotor = NULL;				// currently set motor used for interaction
	numberOfActuators = 0;				// number of motors that have been initialized
	numberOfEncoders = 0;				// number of rotary encoders that have been initialized
	numberOfSteppers = 0;				// number of steppers that have been initialized
	setupDone = false;					// flag to indicate a finished setup (used in stepperloop())
	enabled = false;					// disabled until explicitly enabled
}

void Controller::enable() {
	if (setupDone) {
		for (int i = 0;i<numberOfActuators;i++) { 
			getActuator(i)->enable();
			// give it a break to not overload power supply by switching on all steppers at the same time
			delay(10);
		}
		enabled = true;
	}
}

void Controller::disable() {
	if (setupDone) {
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
		ActuatorSetupData* thisActuatorSetup = &actuatorSetup[i];
		ActuatorIdentifier id = thisActuatorSetup->id;
		thisActuatorSetup->print();
		for (int j = 0;j<numberOfServos;j++) {
			ServoSetupData* thisServoSetup = &servoSetup[j];
			if (thisServoSetup->id == id) {
				logger->print(F("   "));
				thisServoSetup->print();
			}			
		}
		for (int j = 0;j<numberOfSteppers;j++) {
			StepperSetupData* thisStepperSetup = &stepperSetup[j];
			if (thisStepperSetup->id == id) {
				logger->print(F("   "));
				thisStepperSetup->print();
			}
		}
		for (int j = 0;j<numberOfEncoders;j++) {
			RotaryEncoderSetupData* thisEncoderSetup = &encoderSetup[j];
			if (thisEncoderSetup->id == id) {
				logger->print(F("   "));
				thisEncoderSetup->print();
			}
		}

	}
	
	logger->println(F("ACTUATOR CONFIG"));
	memory.println();
}

bool Controller::setuped() {
	return numberOfActuators > 0;
}

bool Controller::setup() {
	bool result = true;;
	numberOfActuators = 0;
	numberOfSteppers = 0;
	numberOfEncoders = 0;
	numberOfServos = 0;

	if (memory.persMem.logSetup) {
		logger->println(F("--- com to I2C bus"));
		logger->print(F("    "));
		int devices = doI2CPortScan(logger);
		logger->print(F("    "));
		logger->print(devices);
		if (devices != 5)
			logger->println(F(" devices found, 5 expected"));
		else
			logger->println(F(" devices found, ok"));
	}

	if (memory.persMem.logSetup) {
		logger->println(F("--- com to servo"));
	}
	
	// Herkulex servos are connected via Serial1
	HerkulexServoDrive::setupCommunication();

	if (memory.persMem.logSetup) {
		logger->println(F("--- initializing actuators"));
	}

	// initialize wrist encoder first, since we need to reprogrammes its I2C address
	logger->println(F("    reprogramme one sensors I2C address"));
	for (numberOfActuators = 0;numberOfActuators<MAX_ACTUATORS;numberOfActuators++) {
		ActuatorConfig* thisActuatorConfig = &(memory.persMem.armConfig[numberOfActuators]);
		RotaryEncoder* encoder = &encoders[numberOfEncoders];	
		if (thisActuatorConfig->actuatorType == STEPPER_ENCODER_TYPE) {
			if (encoderSetup[numberOfEncoders].programmI2CAddress) {
				encoder->setup(&actuatorConfigType[MAX_ACTUATORS-1-numberOfActuators], &(thisActuatorConfig->config.stepperArm.encoder), &(encoderSetup[numberOfEncoders]));
			}
			numberOfEncoders++;
		}
	}

	numberOfEncoders = 0; // restart
	for (numberOfActuators = 0;numberOfActuators<MAX_ACTUATORS;numberOfActuators++) {
		if (memory.persMem.logSetup) {
			logger->print(F("--- setup "));
			logActuator((ActuatorIdentifier)numberOfActuators);
			logger->println(F(" ---"));
		}

		Actuator* thisActuator = &actuators[numberOfActuators];
		ActuatorSetupData* thisActuatorSetup= &actuatorSetup[numberOfActuators];
		ActuatorConfig* thisActuatorConfig = &(memory.persMem.armConfig[numberOfActuators]);
		switch (thisActuatorConfig->actuatorType) {
			case SERVO_TYPE: {
				if (numberOfServos >= MAX_SERVOS)
					logFatal(F("too many servos"));

				HerkulexServoDrive* servo = &servos[numberOfServos];
				servo->setup( &(memory.persMem.armConfig[numberOfActuators].config.servoArm.servo), &(servoSetup[numberOfServos])); 
				thisActuator->setup(thisActuatorConfig, thisActuatorSetup, servo);
				numberOfServos++;

				if (thisActuator->hasStepper())
					logFatal(F("misconfig: stepper!"));
				if (thisActuator->hasEncoder())
					logFatal(F("misconfig: encoder!"));
				if (!thisActuator->hasServo())
					logFatal(F("misconfig: no servo"));

				break;
			}
			case STEPPER_ENCODER_TYPE: {
				if (numberOfEncoders >= MAX_ENCODERS)
					logFatal(F("too many encoders"));
				if (numberOfSteppers >= MAX_STEPPERS)
					logFatal(F("too many steppers"));

				RotaryEncoder* encoder = &encoders[numberOfEncoders];
				GearedStepperDrive* stepper = &steppers[numberOfSteppers];

				encoder->setup(&actuatorConfigType[MAX_ACTUATORS-1-numberOfActuators], &(thisActuatorConfig->config.stepperArm.encoder), &(encoderSetup[numberOfEncoders]));
				stepper->setup(&(thisActuatorConfig->config.stepperArm.stepper), &actuatorConfigType[MAX_ACTUATORS-1-numberOfActuators], &(stepperSetup[numberOfSteppers]));
				thisActuator->setup(thisActuatorConfig, thisActuatorSetup, stepper, encoder);

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
						stepper.setMeasuredAngle(angle); // tell stepper that this is a measured position		
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
	analogReference(EXTERNAL); // use voltage at AREF Pin as reference
	
	setupDone = true;
	if (memory.persMem.logSetup) {
		logger->println(F("setup done"));
	}

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
		digitalWrite(POWER_SUPPLY_STEPPER_PIN, LOW);	// start with stepper to not confuse servo by impulse
		delay(100);										// small break to give power supply to become stable
		digitalWrite(POWER_SUPPLY_SERVO_PIN, LOW);		// servo power supply is harmless, just 2x 450mA
		delay(100);										// again small break to give it time to become stable
	}
	isPowered = on;
}

bool Controller::powered() {
	return isPowered;
}

void Controller::stepperLoop() {
	if (setupIsDone()) {
		// call all stepper loops as often as you can, this makes the timing of the movement precise and smooth
		for (uint8_t i = 0;i<numberOfSteppers;i++) {
			steppers[i].loop(millis());
		}
	}
}

void Controller::loop() {
	stepperLoop(); // send impulses to steppers
	
	// loop that checks the proportional knob	
	if (currentMotor != NULL) {
		if (adjustWhat == ADJUST_MOTOR_BY_KNOB) {
			if (motorKnobTimer.isDue_ms(MOTOR_KNOB_SAMPLE_RATE)) {
				// fetch value of potentiometer, returns 0..1024 representing 0..2.56V
				int16_t adcValue = analogRead(MOTOR_KNOB_PIN);

				// compute angle out of adcDiff, potentiometer turns between 0°..270°
				float angle = (float(adcValue-512)/512.0) * (270.0 / 2.0);			
				static float lastAngle = 0;

				if ( (abs(adcValue-512)<500) &&  (lastAngle != 0) && abs(angle-lastAngle)>0.1) {
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
	if (servoLoopTimer.isDue_ms(SERVO_SAMPLE_RATE)) {
		uint32_t now = millis();
		for (int i = 0;i<MAX_SERVOS;i++) {
			servos[i].loop(now);			
		}
	}
	
	// fetch the angles from the encoders and tell the stepper controller

	if (encoderLoopTimer.isDue_ms(ENCODER_SAMPLE_RATE)) {
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
					bool commOk = encoders[encoderIdx].getNewAngleFromSensor(); // measure the encoder's angle

					if (commOk) {						
						float encoderAngle = encoders[encoderIdx].getAngle();
						/*
						logger->print("enc(");
						logger->print(encoderIdx);
						logger->print(")=");
						logger->print(encoderAngle,2);
						*/
						stepper.setMeasuredAngle(encoderAngle); // and tell Motordriver
						/*
														logger->print("EM(is=");
														logger->print(currentAngle,1);
														logger->print(" enc=");
														logger->println(encoderAngle,1);
														logger->print(" after=");
														logger->print(stepper.getCurrentAngle());
														logger->print(" tobe=");
														logger->println(stepper.getToBeAngle(),1);
						*/

					}
					else { // encoder not plausible ignore it and use last position
						stepper.setMeasuredAngle(currentAngle);
					}
				} else {
					stepper.setMeasuredAngle(currentAngle);				
				}
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
			logger->print(measuredAngle);
			logger->print("(");
				
			measuredAngle = encoder.getRawSensorAngle();
			logger->print(measuredAngle);
			logger->print(") ");
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