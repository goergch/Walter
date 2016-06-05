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

#define ADJUST_KP 1
#define ADJUST_KI 2
#define ADJUST_KD 3
#define MOVE_MOTOR 4

extern Controller motors;

TimePassedBy servoLoopTimer;
TimePassedBy encoderLoopTimer;

bool interruptSemaphore = false;
int adjustWhat = ADJUST_KP;

uint32_t loopCounter = 0;

void stepperLoopInterrupt() {
	interruptSemaphore = true;
	motors.stepperLoop();
	interruptSemaphore = false;
}

Controller::Controller()
{
	currentMotor = NULL;				// currently set motor used for interaction
	numberOfMotors = 0;					// number of motors that have been initialized
	numberOfEncoders = 0;
	interactiveOn = false;
}

void Controller::setup() {
	numberOfMotors = 0;
	numberOfEncoders = 0;

	wristMotor.setup(0); // wrist
	numberOfMotors++;

	// initialize stepper and encoder
	stepper[0].setup(1 /* actuatorNumber */); // wrist nick 
	numberOfMotors++;

	encoders[0].setup(1 /* actuatorNumber */); // wrist nick
	stepper[0].setRotaryEncoder(encoders[0]);
	numberOfEncoders++;
	encoders[1].setup(2 /* actuatorNumber */); // wrist turn
	stepper[1].setRotaryEncoder(encoders[1]);
	numberOfEncoders++;
	
	// get measurement of encoder and ensure that it is plausible
	bool encoderCheckOk = checkEncoders();
	
	// knob control of a motor uses a poti that is measured with the internal adc
	analogReference(EXTERNAL); // use voltage at AREF Pin as reference
	
	// steppers are controlled by a timer that triggers every few us and moves a step
	Timer1.initialize(STEPPER_SAMPLE_RATE_US); // set a timer of length by microseconds
	Timer1.attachInterrupt( stepperLoopInterrupt ); // attach the service routine here
	
	printStepperConfiguration();
}

Actuator* Controller::getMotor(int motorNumber) {
	if (motorNumber == 0)
		return &wristMotor;
	else
		return &stepper[motorNumber-1];
}


void Controller::printStepperConfiguration() {
	Serial.println(F("Stepper Configuration"));
	for (int i = 1;i<numberOfMotors;i++) {
		stepper[i].printConfiguration();		
	}
}
void Controller::printMenuHelp() {
	Serial.println(F("MotorDriver Legs"));
	Serial.println(F("0       - consider all motors"));
	Serial.println(F("1..6    - consider motor"));
	
	Serial.println(F("+/-     - adjust"));
	Serial.println(F("p/i/d   - adjust PID tuning param"));
	Serial.println(F("m       - adjust motor"));
	Serial.println(F("</>     - set motor min/max"));
	Serial.println(F("n       - set nullposition"));

	Serial.println(F("h       - help"));
	Serial.println(F("esc     - exit"));
	
	
	memory.println();
	
	printEncoderAngles();
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
					currentMotor = getMotor(inputChar-'1');
					if (currentMotor != NULL) {
						Serial.print(F("considering motor "));
						Serial.println(currentMotor->getActuatorNumber()+1);
					}

					break;
				case 'n': 
					if (currentMotor->hasEncoder()) {
						Serial.println(F("setting null"));
						float sample[4], avr, variance;
						if (currentMotor->getEncoder()->fetchRawSample(4, sample, avr, variance)) {
							currentMotor->getEncoder()->setNullAngle(avr);
						} else {
							Serial.println(F("sample not ok"));
						}
					} else {
						Serial.println(F("no encoder"));
					}
					break;
				case '<':
				case '>': {
					bool isMax = (inputChar=='>');
					Serial.print(F("setting "));
					Serial.println(isMax?F("max"):F("min"));
					if (currentMotor->hasEncoder()) {
						float avr, variance;
						if (currentMotor->getEncoder()->fetchNormalSample(avr, variance)) {
							if (isMax)
								currentMotor->setMaxAngle(avr);
							else
								currentMotor->setMinAngle(avr);
						} else {
							Serial.println(F("sample not ok"));
						}
					}

					break;
				}
				case 'p':
					Serial.println(F("adjusting Kp"));
					adjustWhat = ADJUST_KP;
					break;
				case 'i':
					Serial.println(F("adjusting Ki"));
					adjustWhat = ADJUST_KI;
					break;
				case 'd':
					Serial.print(F("adjusting Kd "));
					adjustWhat = ADJUST_KD;
					break;
				case 'm':
					Serial.print(F("moving motor"));
					adjustWhat = MOVE_MOTOR;
					break;

				case '+':
				case '-':
					if (currentMotor != NULL) {
						float adjust = (inputChar=='+')?0.1:-0.1;

						switch (adjustWhat){
							case ADJUST_KP:
								memory.persMem.armConfig[currentMotor->getActuatorNumber()].pivKp +=adjust;
								Serial.print("Kp=");
								Serial.println(memory.persMem.armConfig[currentMotor->getActuatorNumber()].pivKp);
								break;
							case ADJUST_KI:
								memory.persMem.armConfig[currentMotor->getActuatorNumber()].pivKi +=adjust;
								Serial.print("Ki=");
								Serial.println(memory.persMem.armConfig[currentMotor->getActuatorNumber()].pivKi);
								break;
							case ADJUST_KD:
								memory.persMem.armConfig[currentMotor->getActuatorNumber()].pivKd +=adjust;
								Serial.print("Kd=");
								Serial.println(memory.persMem.armConfig[currentMotor->getActuatorNumber()].pivKd);

								break;
							case MOVE_MOTOR:
								memory.persMem.armConfig[currentMotor->getActuatorNumber()].pivKd +=adjust;
								Serial.print("pos=");
								Serial.println(currentMotor->getCurrentAngle());
								break;
							default:
								break;
						}

						currentMotor->setPIVParams();
						currentMotor->getPIV()->print();
						Serial.println();
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
	loopCounter++;
	static uint16_t smallestSampleRate = min(ENCODER_SAMPLE_RATE,min(MOTOR_KNOB_SAMPLE_RATE,SERVO_SAMPLE_RATE));
	
	if (currentMotor != NULL) {
		if (motorKnobTimer.isDue_ms(MOTOR_KNOB_SAMPLE_RATE)) {
			// fetch value of potentiometer, returns 0..1024 representing 0..2.56V
			int16_t adcValue = analogRead(MOTOR_KNOB_PIN);

			// compute angle out of adcDiff, potentiometer turns between 0°..270°
			float angle = float(adcValue-512)/512.0*135.0;			
			
			static float lastAngle = 0;
			if (abs(angle-lastAngle)>0.1) {
				lastAngle = angle;
				// turn to defined angle according to the predefined sample rate
				while (interruptSemaphore) delayMicroseconds(STEPPER_SAMPLE_RATE_US/2); // ensure that a steppers does not move at this very moment

				currentMotor->setAngle(angle,MOTOR_KNOB_SAMPLE_RATE);	
			}
		}
	};
	
	if (servoLoopTimer.isDue_ms(SERVO_SAMPLE_RATE)) {
		wristMotor.loop(millis());
	}
	
	if (encoderLoopTimer.isDue_ms(ENCODER_SAMPLE_RATE)) {
		// fetch encoder values and tell the stepper measure (numberOfMotors includes the servo, so start from 2)
		for (int i = 0;i<numberOfEncoders;i++) {
			encoders[i].fetchAngle(); // measure the encoder's angle
			float measuredAngle = encoders[i].getAngle();
			// stepper[i-1].setMeasuredAngle(measuredAngle); // and tell Motordriver 
		}		
#ifdef DEBUG_ENCODERS		
		printEncoderAngles();
#endif
	}
	if (interactiveOn)
		interactiveLoop();
}

void Controller::printEncoderAngles() {
	Serial.print(F("encoders={"));
	for (int i = 0;i<numberOfMotors;i++) {
		Serial.print(i);
		Serial.print("=");

		float measuredAngle = encoders[i].getAngle();
		Serial.print(measuredAngle);		
		Serial.print("(");
		
		measuredAngle = encoders[i].getRawAngle();
		Serial.print(measuredAngle);
		Serial.print(")");
	}
	Serial.println("}");
}

void Controller::stepperLoop()
{
	uint32_t now = millis();
	for (uint8_t i = 0;i<numberOfMotors-1;i++) {
		stepper[i].loop(now);
	}
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