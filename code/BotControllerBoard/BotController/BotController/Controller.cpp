/* 
* Motors.cpp
*
* Created: 22.04.2016 19:26:44
* Author: JochenAlt
*/

#include "Arduino.h"
#include "utilities.h"
#include "setup.h"
#include "BotMemory.h"
#include "Controller.h"
#include <avr/wdt.h>
#include "GearedStepperDrive.h"
#include "RotaryEncoder.h"

#define ADJUST_MOTOR_MANUALLY 1
#define ADJUST_MOTOR_BY_KNOB_WITHOUT_FEEDBACK 2
#define ADJUST_MOTOR_BY_KNOB_WITH_FEEDBACK 3
		

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
	interactiveOn = false;				// interactive mode is off by default
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

bool Controller::selectActuator(uint8_t no) {
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
		ActuatorId id = thisActuatorSetup->id;
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
	if (logSetup) {
		logger->println(F("--- com to servo"));
	}

	// Herkulex servos are connected via Serial1
	HerkulexServoDrive::setupCommunication();
		
	// Gripper is a Herkulex servo
	if (logSetup) {
		logger->println(F("--- setup gripper"));
	}
	Actuator* thisActuator = &actuators[numberOfActuators];
	HerkulexServoDrive* servo = &servos[numberOfServos];
	
	ActuatorSetupData* thisActuatorSetup= &actuatorSetup[numberOfActuators];
	ServoSetupData* thisServoSetup = &servoSetup[numberOfServos];
	ActuatorConfig* thisActuatorConfig = &(memory.persMem.armConfig[numberOfActuators]);
	ServoConfig* thisServoConfig = &(memory.persMem.armConfig[numberOfActuators].config.servoArm.servo);
	servo->setup( thisServoConfig, thisServoSetup); // wrist
	thisActuator->setup(thisActuatorConfig, thisActuatorSetup, servo);
	
	numberOfServos++;
	numberOfActuators++;

	// Hand is a Herkulex servo
	if (logSetup) {
		logger->println(F("--- setup hand"));
	}
	thisActuator = &actuators[numberOfActuators];
	servo = &servos[numberOfServos];
	thisActuatorSetup= &actuatorSetup[numberOfActuators];
	thisServoSetup = &servoSetup[numberOfServos];
	thisActuatorConfig = &(memory.persMem.armConfig[numberOfActuators]);
	thisServoConfig = &(memory.persMem.armConfig[numberOfActuators].config.servoArm.servo);
	servo->setup( thisServoConfig, thisServoSetup); 
	thisActuator->setup(thisActuatorConfig, thisActuatorSetup, servo);
	numberOfServos++;
	numberOfActuators++;

	// Wrist Nick is a stepper plus encoder
	if (logSetup) {
		logger->println(F("--- setup ellbow"));
	}
	thisActuator = &actuators[numberOfActuators];
	RotaryEncoder* encoder = &encoders[numberOfEncoders];
	GearedStepperDrive* stepper = &steppers[numberOfSteppers];

	thisActuatorSetup = &actuatorSetup[numberOfActuators];
	RotaryEncoderSetupData* thisEncoderSetup = &encoderSetup[numberOfEncoders];
	StepperSetupData* thisStepperSetup = &stepperSetup[numberOfSteppers];
	thisActuatorConfig = &memory.persMem.armConfig[numberOfActuators];
	
	encoder->setup(&thisActuatorConfig->config.stepperArm.encoder, thisEncoderSetup);
	stepper->setup(&thisActuatorConfig->config.stepperArm.stepper, thisStepperSetup);
	thisActuator->setup(thisActuatorConfig, thisActuatorSetup, stepper, encoder);
	numberOfEncoders++;
	numberOfSteppers++;
	numberOfActuators++;
	
	// Ellbow
	if (logSetup) {
		logger->println(F("--- setup forearm"));
	}
	thisActuator = &actuators[numberOfActuators];
	encoder = &encoders[numberOfEncoders];
	stepper = &stepper[numberOfSteppers];

	thisActuatorConfig = &memory.persMem.armConfig[numberOfActuators];
	thisActuatorSetup = &actuatorSetup[numberOfActuators];
	thisEncoderSetup= &encoderSetup[numberOfEncoders];
	thisStepperSetup = &stepperSetup[numberOfSteppers];

	encoder->setup(&thisActuatorConfig->config.stepperArm.encoder, thisEncoderSetup);
	stepper->setup(&thisActuatorConfig->config.stepperArm.stepper, thisStepperSetup);
	thisActuator->setup(thisActuatorConfig, thisActuatorSetup, stepper, encoder);

	numberOfEncoders++;
	numberOfSteppers++;
	numberOfActuators++;

	// Upperarm  
	if (logSetup) {
		logger->println(F("--- setup upperarm"));
	}
	thisActuator = &actuators[numberOfActuators];
	encoder = &encoders[numberOfEncoders];
	stepper = &stepper[numberOfSteppers];

	thisActuatorConfig = &memory.persMem.armConfig[numberOfActuators];
	thisActuatorSetup = &actuatorSetup[numberOfActuators];
	thisEncoderSetup= &encoderSetup[numberOfEncoders];
	thisStepperSetup = &stepperSetup[numberOfSteppers];

	encoder->setup(&thisActuatorConfig->config.stepperArm.encoder, thisEncoderSetup);
	stepper->setup(&thisActuatorConfig->config.stepperArm.stepper, thisStepperSetup);
	thisActuator->setup(thisActuatorConfig, thisActuatorSetup, stepper, encoder);

	numberOfEncoders++;
	numberOfSteppers++;
	numberOfActuators++;
	
	if (logSetup) {
		logger->println(F("--- check encoders"));
	}
	
	// get measurement of encoder and ensure that it is plausible 
	// (variance of a couple of samples needs to be low)
	bool encoderCheckOk = checkEncoders();
	
	// set measured angle of the actuators and define that angle as current position by setting the movement
	for (int i = 0;i<numberOfActuators;i++) {
		Actuator* actuator = getActuator(i);
		if (actuator->hasEncoder()) {

			RotaryEncoder& encoder = actuator->getEncoder();

			// find corresponding stepper
			if (actuator->hasStepper()) {
				GearedStepperDrive& stepper= actuator->getStepper();
				if (encoder.getConfig().id != stepper.getConfig().id) {
					printActuator(stepper.getConfig().id);
					fatalError(F("encoder and stepper different"));
				} else {
					if (encoder.isOk()) {
						float angle = encoder.getAngle();
						stepper.setMeasuredAngle(angle); // tell stepper that this is a measured position		
						stepper.setAngle(angle,1);	     // define a current movement that ends up at current angle. Prevents uncontrolled startup.
					}
					else  {
						printActuator(stepper.getConfig().id);
						fatalError(F("encoder not ok"));
						result = false;
					}
				}
			} else {
					actuator->printName();
					fatalError(F("encoder has no stepper"));				
					result = false;
			}
		}
	}
	
	if (logSetup) {
		logger->println(F("--- initialize ADC"));
	}
	
	
	// knob control of a motor uses a poti that is measured with the internal adc
	analogReference(EXTERNAL); // use voltage at AREF Pin as reference
	
	setupDone = true;
	return false;
}

Actuator* Controller::getActuator(uint8_t actuatorNumber) {
	if ((actuatorNumber>= 0) && (actuatorNumber<numberOfActuators))
		return &actuators[actuatorNumber];
	else 
		return NULL;
}


void Controller::printMenuHelp() {
	logger->println(F("MotorDriver Legs"));
	logger->println(F("0       - consider all motors"));
	logger->print(F("1..6    - consider motor"));
	if (currentMotor != NULL) {
		logger->print(F("("));
		printActuator(currentMotor->getConfig().id);
		logger->print(F(")"));
	}

	logger->println();		
	logger->println(F("+/-     - adjust"));
	logger->print(F("m       - adjust motor"));
	if (adjustWhat == ADJUST_MOTOR_MANUALLY)
		logger->print(F(" *"));
	logger->println();
	logger->print(F("k       - use delta knob"));
	if (adjustWhat == ADJUST_MOTOR_BY_KNOB_WITHOUT_FEEDBACK)
		logger->print(F(" *"));
	logger->println();
	logger->print(F("K       - use encoder knob"));
	if (adjustWhat == ADJUST_MOTOR_BY_KNOB_WITH_FEEDBACK)
		logger->print(F(" *"));
	logger->println();

	logger->println(F("</>     - set motor min/max"));
	logger->println(F("n       - set nullposition"));
	logger->println(F("e       - enable/disable"));

	logger->println(F("h       - help"));
	logger->println(F("s       - print memory"));

	logger->println(F("esc     - exit"));
	
	logger->println();
	printAngles();
}


void Controller::interactive(bool on) {
	interactiveOn = on;
	if (on)
		printMenuHelp();
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
		delay(10);										// small break to give power supply to become stable
		digitalWrite(POWER_SUPPLY_SERVO_PIN, LOW);		// servo power supply is harmless, just 2x 450mA
		delay(10);										// again small break to give it time to become stable
	}
}


void Controller::interactiveLoop() {
		if (Serial.available()) {
			static char inputChar;
			inputChar = Serial.read();
			switch (inputChar) {
				case '0':
					selectActuator(0);
					logger->println(F("no motor considered"));
					break;
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7': {
					selectActuator(inputChar-'1');
					break;	
				}
				case 'e':
					if (isEnabled()) {
						disable();
						logger->println(F("disabled."));
					} else {
						enable();
						logger->println(F("enabled."));
					}
				break;

				case 'n': 
					logger->println(F("setting null"));
					if (currentMotor->setCurrentAsNullPosition())
						memory.delayedSave();
					else 
						logger->println(F("not successfull"));
					break;
				case '<':
				case '>': {
					bool isMax = (inputChar=='>');
					logger->print(F("setting "));
					logger->println(isMax?F("max"):F("min"));
					if (currentMotor->hasEncoder()) {
						if (!currentMotor->getEncoder().isOk())
							logger->println(F("encoder not calibrated"));
						else {
							float avr, variance;
							if (currentMotor->getEncoder().fetchSample(false,avr, variance)) {
								logger->print(F("avr="));
								logger->println(avr);

								if (isMax)
									currentMotor->setMaxAngle(avr);
								else
									currentMotor->setMinAngle(avr);
								memory.delayedSave();
							} else {
								logger->println(F("sample not ok"));
							}
						}
					}
					if (currentMotor->hasServo()) {
						if (currentMotor->getServo().isOk()) {
							float angle = currentMotor->getCurrentAngle();
							if (isMax)
								currentMotor->setMaxAngle(angle);
							else
								currentMotor->setMinAngle(angle);
							memory.delayedSave();
						}
						else 
							logger->println(F("servo not ok"));
					} 
					break;
					}
				case 's':
					memory.println();
					printConfiguration();
					break;
				case 'k':
					logger->println(F("adjusting motor by knob incrementally"));
					adjustMotor(ADJUST_MOTOR_BY_KNOB_WITHOUT_FEEDBACK);
					// steppers turns even when rotarys are not working. Initialize with 0 angle
					break;
				case 'K':
					logger->println(F("setting motor by knob"));
					adjustMotor(ADJUST_MOTOR_BY_KNOB_WITH_FEEDBACK);
					// steppers turns even when rotarys are not working. Initialize with 0 angle
					break;

				case 'm':
					logger->println(F("adjusting motor manually"));
					adjustMotor(ADJUST_MOTOR_MANUALLY);

				case '+':
				case '-':
					if (currentMotor != NULL) {
						float adjust = (inputChar=='+')?0.1:-0.1;
						changeAngle(adjust, MOTOR_KNOB_SAMPLE_RATE);
						logger->print(F("adjusting by "));
						logger->println(adjust,1);
					}
					break;	
				case 'h':
					printMenuHelp();
					break;
				case '\e':
					// disable old motor
					if (currentMotor != NULL) {
						currentMotor->disable();
						currentMotor = NULL;
					}

					interactiveOn = false;
					return;
				default:
					break;
			} // switch
		} // if (Serial.available())
}


void Controller::stepperLoop() {
	if (setupIsDone()) {
		// loop to be called most often is the stepper loop
		for (uint8_t i = 0;i<numberOfSteppers;i++) {
			steppers[i].loop(millis());
		}
	}
}

void Controller::loop() {

	stepperLoop();
	
	// anything to be stored in epprom?
	memory.loop();
	
	// loop that checks the proportional knob	
	if (currentMotor != NULL) {
		if ((adjustWhat == ADJUST_MOTOR_BY_KNOB_WITHOUT_FEEDBACK) || 
		    (adjustWhat == ADJUST_MOTOR_BY_KNOB_WITH_FEEDBACK)) {
			if (motorKnobTimer.isDue_ms(MOTOR_KNOB_SAMPLE_RATE)) {
				// fetch value of potentiometer, returns 0..1024 representing 0..2.56V
				int16_t adcValue = analogRead(MOTOR_KNOB_PIN);

				// compute angle out of adcDiff, potentiometer turns between 0°..270°
				float angle = float(adcValue-512)/512.0*135.0;			
				static float lastAngle = 0;

				bool doItAbsolute = (adjustWhat == ADJUST_MOTOR_BY_KNOB_WITH_FEEDBACK);
				if (doItAbsolute) {
					
					if ((lastAngle != 0) && abs(angle-lastAngle)>0.5) {
						logger->print(F("knob="));
						logger->println(angle,1);
						
						// turn to defined angle according to the predefined sample rate
						currentMotor->setAngle(angle,MOTOR_KNOB_SAMPLE_RATE);
					}
				} else {
					if ((lastAngle != 0) && abs(angle-lastAngle)>0.5) {
						logger->print(F("adjust by "));
						logger->println(angle-lastAngle,1);
					
						// turn to defined angle according to the predefined sample rate
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
		for (int encoderIdx = 0;encoderIdx<numberOfEncoders;encoderIdx++) {
			if (encoders[encoderIdx].isOk()) {
				bool plausible = encoders[encoderIdx].getNewAngleFromSensor(); // measure the encoder's angle
				if (plausible) {
					float encoderAngle = encoders[encoderIdx].getAngle();
				
					// find corresponding actuator
					ActuatorId actuatorID = encoders[encoderIdx].getConfig().id;
					Actuator* actuator = getActuator(actuatorID);
					if (actuator->hasStepper()) {
						GearedStepperDrive& stepper = actuator->getStepper();						
						// check this
						if (stepper.getConfig().id != actuatorID) {
							printActuator(actuatorID);
							logger->print(actuatorID);
							logger->print(encoderIdx);
							logger->print(stepper.getConfig().id);

							fatalError(F("wrong stepper identified"));
						}
						else {
							float currentAngle = stepper.getCurrentAngle();
							stepper.setMeasuredAngle(encoderAngle); // and tell Motordriver	
							/*
							logger->print("EM(is=");
							logger->print(currentAngle,1);
							logger->print(" enc=");
							logger->println(encoderAngle,1);
							logger->print(" after=");
							logger->print(steppers[stepperIdx].getCurrentAngle());
							logger->print(" tobe=");
							logger->println(steppers[stepperIdx].getToBeAngle(),1);
							*/
						}
					}
				}
			}
		}
	}		

	if (logEncoder) 
		printAngles();

	if (interactiveOn)
		interactiveLoop();
}

void Controller::printAngles() {
	
	logger->print(F("angles{"));
	for (int actNo = 0;actNo<numberOfActuators;actNo++) {
		Actuator* actuator=  getActuator(actNo);
		printActuator(actuator->getConfig().id);
		
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

bool  Controller::checkEncoders() {
	bool ok = true;
	for (int i = 0;i<numberOfEncoders;i++) {
		wdt_reset(); // this might take longer
		float variance = encoders[i].checkEncoderVariance();
		if (!encoders[i].isOk())
			ok = false;
	}

	return ok;
}