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
#include "TimerOne.h"
#include "GearedStepperDrive.h"
#include "RotaryEncoder.h"

#define ADJUST_MOTOR_MANUALLY 1
#define ADJUST_MOTOR_BY_KNOB 2
#define ADJUST_MOTOR_ANGLE_ABS_BY_KNOB 3
		

extern Controller controller;
TimePassedBy servoLoopTimer;
TimePassedBy encoderLoopTimer;
uint8_t adjustWhat = ADJUST_MOTOR_MANUALLY;


Controller::Controller()
{
	currentMotor = NULL;				// currently set motor used for interaction
	numberOfActuators = 0;					// number of motors that have been initialized
	numberOfEncoders = 0;
	numberOfSteppers = 0;
	interactiveOn = false;
}

void Controller::printSetupConfiguration() {
	Serial.println(F("ACTUATOR SETUP"));
	for (int i = 0;i<numberOfActuators;i++) {
		ActuatorSetupData* thisActuatorSetup = &actuatorSetup[i];
		ActuatorId id = thisActuatorSetup->id;
		actuatorSetup->print();
		for (int j = 0;j<numberOfServos;j++) {
			ServoSetupData* thisServoSetup = &servoSetup[j];
			if (thisServoSetup->id == id) {
				Serial.print(F("   "));
				thisServoSetup->print();
			}			

		}
		for (int j = 0;j<numberOfSteppers;j++) {
			StepperSetupData* thisStepperSetup = &stepperSetup[j];
			if (thisStepperSetup->id == id) {
				Serial.print(F("   "));
				thisStepperSetup->print();
			}
		}
		for (int j = 0;j<numberOfEncoders;j++) {
			RotaryEncoderSetupData* thisEncoderSetup = &encoderSetup[j];
			if (thisEncoderSetup->id == id) {
				Serial.print(F("   "));
				thisEncoderSetup->print();
			}
		}

		for (int j = 0;j<numberOfServos;j++) {
			ServoSetupData* thisServoSetup = &servoSetup[j];
			if (thisServoSetup->id == id) {
				Serial.print(F("   "));
				thisServoSetup->print();
			}
		}
	}
	
	Serial.println(F("ACTUATOR CONFIG"));
	memory.println();
}

void Controller::setup() {
	numberOfActuators = 0;
	numberOfSteppers = 0;
	numberOfEncoders = 0;
	numberOfServos = 0;

	// Herkulex servos are connected via Serial1
	HerkulexServoDrive::setupCommunication();
	
	// Gripper is a Herkulex servo
#ifdef DEBUG_SETUP	
	Serial.println(F("--- setup gripper"));
#endif
	Actuator* thisActuator = &actuators[numberOfActuators];
	HerkulexServoDrive* servo = &servos[numberOfServos];
	
	ActuatorSetupData* thisActuatorSetup= &actuatorSetup[numberOfActuators];
	ServoSetupData* thisServoSetup = &servoSetup[numberOfServos];
	ActuatorConfigurator* thisActuatorConfig = &(memory.persMem.armConfig[numberOfActuators]);
	ServoConfig* thisServoConfig = &(memory.persMem.armConfig[numberOfActuators].config.servoArm.servo);
	servo->setup( thisServoConfig, thisServoSetup); // wrist
	thisActuator->setup(thisActuatorConfig, thisActuatorSetup, servo);
	
	numberOfServos++;
	numberOfActuators++;

	// Hand is a Herkulex servo
#ifdef DEBUG_SETUP
	Serial.println(F("--- setup hand"));
#endif
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
#ifdef DEBUG_SETUP
	Serial.println(F("--- setup ellbow"));
#endif
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
	
	// Wrist Turn 
#ifdef DEBUG_SETUP
	Serial.println(F("--- setup forearm"));
#endif
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

	// enable all Herkulex Servos (do it now, since they need a lot of time for startup)
	for (int i = 0;i<numberOfEncoders;i++) {
		servos[i].enable();
	}
	
	// get measurement of encoder and ensure that it is plausible
	bool encoderCheckOk = checkEncoders();
	
	// set measured angle of the actuators and define that angle as current position by setting the movement
	for (int i = 0;i<numberOfActuators;i++) {
		Actuator& actuator = getActuator(i);
		if (actuator.hasEncoder()) {

			RotaryEncoder& encoder = actuator.getEncoder();

			// find corresponding stepper
			if (actuator.hasStepper()) {
				GearedStepperDrive& stepper= actuator.getStepper();
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
					}
				}
			} else {
					actuator.printName();
					fatalError(F("encoder has no stepper"));				
			}
		}
	}
	
	
	// knob control of a motor uses a poti that is measured with the internal adc
	analogReference(EXTERNAL); // use voltage at AREF Pin as reference
}

Actuator& Controller::getActuator(uint8_t actuatorNumber) {
	return actuators[actuatorNumber];
}


void Controller::printMenuHelp() {
	Serial.println(F("MotorDriver Legs"));
	Serial.println(F("0       - consider all motors"));
	Serial.print(F("1..6    - consider motor"));
	if (currentMotor != NULL) {
		Serial.print(F("("));
		printActuator(currentMotor->getConfig().id);
		Serial.print(F(")"));
	}
	Serial.println();		
	Serial.println(F("+/-     - adjust"));
	Serial.print(F("m       - adjust motor"));
	if (adjustWhat == ADJUST_MOTOR_MANUALLY)
		Serial.print(F(" *"));
	Serial.println();
	Serial.print(F("k       - use delta knob"));
	if (adjustWhat == ADJUST_MOTOR_BY_KNOB)
		Serial.print(F(" *"));
	Serial.println();
	Serial.print(F("K       - use encoder knob"));
	if (adjustWhat == ADJUST_MOTOR_ANGLE_ABS_BY_KNOB)
		Serial.print(F(" *"));
	Serial.println();

	Serial.println(F("</>     - set motor min/max"));
	Serial.println(F("n       - set nullposition"));

	Serial.println(F("h       - help"));
	Serial.println(F("s       - print memory"));

	Serial.println(F("esc     - exit"));
	
	Serial.println();
	printAngles();
}


void Controller::interactive(bool on) {
	interactiveOn = on;
	if (on)
		printMenuHelp();
}

void Controller::interactiveLoop() {
		if (Serial.available()) {
			static char inputChar;
			inputChar = Serial.read();
			switch (inputChar) {
				case '0':
					currentMotor = NULL;
					Serial.println(F("no motor considered"));
					break;
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':

					currentMotor = &getActuator(inputChar-'1');
					if (currentMotor != NULL) {
						Serial.print(F("considering "));
						currentMotor->printName();
						Serial.println();
					}

					break;
				case 'n': 
					Serial.println(F("setting null"));
					if (currentMotor->setCurrentAsNullPosition())
						memory.delayedSave();
					else 
						Serial.println(F("not successfull"));
					break;
				case '<':
				case '>': {
					bool isMax = (inputChar=='>');
					Serial.print(F("setting "));
					Serial.println(isMax?F("max"):F("min"));
					if (currentMotor->hasEncoder()) {
						if (!currentMotor->getEncoder().isOk())
							Serial.println(F("encoder not calibrated"));
						else {
							float avr, variance;
							if (currentMotor->getEncoder().fetchSample(false,avr, variance)) {
								Serial.print(F("avr="));
								Serial.println(avr);

								if (isMax)
									currentMotor->setMaxAngle(avr);
								else
									currentMotor->setMinAngle(avr);
								memory.delayedSave();
							} else {
								Serial.println(F("sample not ok"));
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
							Serial.println(F("servo not ok"));
					} 
					break;
					}
				case 's':
					memory.println();
					printSetupConfiguration();
					break;
				case 'k':
					Serial.println(F("adjusting motor by knob incrementally"));
					adjustWhat = ADJUST_MOTOR_BY_KNOB;
					// steppers turns even when rotarys are not working. Initialize with 0 angle
					break;
				case 'K':
					Serial.println(F("setting motor by knob"));
					adjustWhat = ADJUST_MOTOR_ANGLE_ABS_BY_KNOB;
					// steppers turns even when rotarys are not working. Initialize with 0 angle
					break;

				case 'm':
					Serial.println(F("adjusting motor manually"));
					adjustWhat = ADJUST_MOTOR_MANUALLY;
					break;

				case '+':
				case '-':
					if (currentMotor != NULL) {
						float adjust = (inputChar=='+')?0.1:-0.1;
						switch (adjustWhat){
							case ADJUST_MOTOR_MANUALLY: {
									adjust = (inputChar=='+')?1.0:-1.0;
									currentMotor->changeAngle(adjust, MOTOR_KNOB_SAMPLE_RATE);
									Serial.print("adjusting by ");
									Serial.println(adjust,1);
								}
								break;
							default:
								break;
						}
						
					}
					break;
				case 'h':
					printMenuHelp();
					break;
				case '\e':
					interactiveOn = false;
					return;
				default:
					break;
			} // switch
		} // if (Serial.available())
}


void Controller::loop() {

	// loop to be called most often is the stepper loop
	for (uint8_t i = 0;i<numberOfSteppers;i++) {
		steppers[i].loop(millis());
	}	
	
	// anything to be stored in epprom?
	memory.loop();
	
	// loop that checks the proportional knob	
	if (currentMotor != NULL) {
		if ((adjustWhat == ADJUST_MOTOR_BY_KNOB) || (adjustWhat == ADJUST_MOTOR_ANGLE_ABS_BY_KNOB)) {
			if (motorKnobTimer.isDue_ms(MOTOR_KNOB_SAMPLE_RATE)) {
				// fetch value of potentiometer, returns 0..1024 representing 0..2.56V
				int16_t adcValue = analogRead(MOTOR_KNOB_PIN);

				// compute angle out of adcDiff, potentiometer turns between 0°..270°
				float angle = float(adcValue-512)/512.0*135.0;			
				static float lastAngle = 0;

				bool doItAbsolute = (adjustWhat == ADJUST_MOTOR_ANGLE_ABS_BY_KNOB);
				if (doItAbsolute) {
					
					if ((lastAngle != 0) && abs(angle-lastAngle)>0.5) {
						Serial.print(F("knob="));
						Serial.println(angle,1);
						
						// turn to defined angle according to the predefined sample rate
						currentMotor->setAngle(angle,MOTOR_KNOB_SAMPLE_RATE);
					}
				} else {
					if ((lastAngle != 0) && abs(angle-lastAngle)>0.5) {
						Serial.print(F("adjust by "));
						Serial.println(angle-lastAngle,1);
					
						// turn to defined angle according to the predefined sample rate
						currentMotor->changeAngle(angle-lastAngle,MOTOR_KNOB_SAMPLE_RATE);	
					}
				}

				lastAngle = angle;				
			}
		}
	};
	
	// update the servos
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
					Actuator& actuator = getActuator(actuatorID);
					if (actuator.hasStepper()) {
						GearedStepperDrive& stepper = actuator.getStepper();						
						// check this
						if (stepper.getConfig().id != actuatorID) {
							printActuator(actuatorID);
							Serial.print(actuatorID);
							Serial.print(encoderIdx);
							Serial.print(stepper.getConfig().id);

							fatalError(F("wrong stepper identified"));
						}
						else {
							float currentAngle = stepper.getCurrentAngle();
							stepper.setMeasuredAngle(encoderAngle); // and tell Motordriver	
							/*
							Serial.print("EM(is=");
							Serial.print(currentAngle,1);
							Serial.print(" enc=");
							Serial.println(encoderAngle,1);
							Serial.print(" after=");
							Serial.print(steppers[stepperIdx].getCurrentAngle());
							Serial.print(" tobe=");
							Serial.println(steppers[stepperIdx].getToBeAngle(),1);
							*/
						}
					}
				}
			}
		}		

#ifdef DEBUG_ENCODERS		
		printAngles();
#endif
	}

	if (interactiveOn)
		interactiveLoop();
}

void Controller::printAngles() {
	
	Serial.print(F("angles{"));
	for (int actNo = 0;actNo<numberOfActuators;actNo++) {
		Actuator& actuator=  getActuator(actNo);
		printActuator(actuator.getConfig().id);
		
		if (actuator.hasEncoder()) {
			Serial.print(F(" enc="));

			RotaryEncoder& encoder = actuator.getEncoder();
			float measuredAngle = encoder.getAngle();
			Serial.print(measuredAngle);
			Serial.print("(");
				
			measuredAngle = encoder.getRawSensorAngle();
			Serial.print(measuredAngle);
			Serial.print(") ");
		}
		
		if (actuator.hasServo()) {
			Serial.print(F(" srv="));
			HerkulexServoDrive& servo= actuator.getServo();
			float measuredAngle = servo.getCurrentAngle();
			Serial.print(measuredAngle);
			Serial.print("(");
			
			measuredAngle = servo.getRawAngle();
			Serial.print(measuredAngle);
			Serial.print(") ");
		}
	}
		
		
	Serial.println("}");
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