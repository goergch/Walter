/* 
* Motors.cpp
*
* Created: 22.04.2016 19:26:44
* Author: JochenAlt
*/

#include "Arduino.h"
#include "setup.h"
#include "BotMemory.h"
#include "Motors.h"
#include <avr/wdt.h>
#include "TimerOne.h"

#define ADJUST_KP 1
#define ADJUST_KI 2
#define ADJUST_KD 3

extern Motors motors;

TimePassedBy servoLoopTimer;
TimePassedBy encoderLoopTimer;

bool interruptSemaphore = false;
int adjustPIDParam = ADJUST_KP;

void stepperLoopInterrupt() {
	interruptSemaphore = true;
	motors.stepperLoop();
	interruptSemaphore = false;
}

Motors::Motors()
{
	currentMotor = NULL;				// currently set motor used for interaction
	numberOfMotors = 0;					// number of motors that have been initialized
	interactiveOn = false;
}

void Motors::setup() {
	numberOfMotors = 0;

	wristMotor.setup(0); // wrist
	numberOfMotors++;

	stepper[0].setup(1); // wrist nick 
	// encoder[0].setup(1); // 
	numberOfMotors++;
	
	// knob control of a motor uses a poti that is measured with the internal adc
	analogReference(EXTERNAL); // use voltage at AREF Pin as reference
	
	// steppers are controlled by a timer that triggers every few us and moves a step
	Timer1.initialize(STEPPER_SAMPLE_RATE_US); // set a timer of length by microseconds
	Timer1.attachInterrupt( stepperLoopInterrupt ); // attach the service routine here
}

MotorDriver* Motors::getMotor(int motorNumber) {
	if (motorNumber == 0)
		return &wristMotor;
	else
		return &stepper[motorNumber-1];
}

void Motors::printMenuHelp() {
	Serial.println(F("MotorDriver Legs"));
	Serial.println(F("0       - consider all motors"));
	Serial.println(F("1..6    - consider motor"));
	Serial.println(F("*/'     - amend nullposition"));
	
	Serial.println(F("+/-     - adjust PID param"));
	Serial.println(F("p/i/d   - set PID tuning param"));

	Serial.println(F("h       - help"));
	Serial.println(F("esc     - exit"));
	
	memory.println();
}


void Motors::interactive(bool on) {
	interactiveOn = on;
	if (on)
		printMenuHelp();
}

void Motors::interactiveLoop() {
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
						Serial.println(currentMotor->getMotorNumber()+1);
					}

					break;
				case '*':
				case '\'':
					currentMotor->addToNullPosition((inputChar=='+')?1:-1);
					break;
				case 'p':
					Serial.println(F("adjusting Kp"));
					adjustPIDParam = ADJUST_KP;
					break;
				case 'i':
					Serial.println(F("adjusting Ki"));
					adjustPIDParam = ADJUST_KI;
					break;
				case 'd':
					Serial.print(F("adjusting Kd "));
					adjustPIDParam = ADJUST_KD;
					break;
				case '+':
				case '-':
					if (currentMotor != NULL) {
						float adjust = (inputChar=='+')?0.1:-0.1;

						switch (adjustPIDParam){
							case ADJUST_KP:
								memory.persistentMem.motorConfig[currentMotor->getMotorNumber()].pivKp +=adjust;
								Serial.print("Kp=");
								Serial.println(memory.persistentMem.motorConfig[currentMotor->getMotorNumber()].pivKp);
								break;
							case ADJUST_KI:
								memory.persistentMem.motorConfig[currentMotor->getMotorNumber()].pivKi +=adjust;
								Serial.print("Ki=");
								Serial.println(memory.persistentMem.motorConfig[currentMotor->getMotorNumber()].pivKi);
								break;
							case ADJUST_KD:
								memory.persistentMem.motorConfig[currentMotor->getMotorNumber()].pivKd +=adjust;
								Serial.print("Kd=");
								Serial.println(memory.persistentMem.motorConfig[currentMotor->getMotorNumber()].pivKd);

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

void Motors::loop() {
	if (currentMotor != NULL) {
		if (motorKnobTimer.isDue_ms(MOTOR_KNOB_SAMPLE_RATE)) {
			// fetch value of potentiometer, returns 0..1024 representing 0..2.56V
			int16_t adcValue = analogRead(MOTOR_KNOB_PIN);
			// compute angle out of adcDiff, potentiometer turns between 0°..270°
			float angle = float(adcValue-512)/512.0*135.0;			
			// turn to defined angle according to the predefined sample rate
			while (interruptSemaphore) delayMicroseconds(100); // ensure that a steppers does not move at this very moment
			currentMotor->setAngle(angle,MOTOR_KNOB_SAMPLE_RATE);
		}
	};
	
	if (servoLoopTimer.isDue_ms(SERVO_SAMPLE_RATE)) {
		wristMotor.loop();
	}
	
	/*	
	if (encoderLoopTimer.isDue_ms(ENCODER_SAMPLE_RATE)) {
		// fetch encoder values and tell the stepper measure
		for (int i = 1;i<=numberOfMotors;i++) {
			encoders[i-1].fetchAngle(); // measure the encoder's angle
			stepper[i-1].setMeasuredAngle(encoders[i-1].getAngle()); // and tell Motordriver 
		}		
	}
	*/
	
	if (interactiveOn)
		interactiveLoop();
}

void Motors::stepperLoop()
{
	for (uint8_t i = 0;i<numberOfMotors-1;i++) {
		stepper[i].loop();
	}
}
