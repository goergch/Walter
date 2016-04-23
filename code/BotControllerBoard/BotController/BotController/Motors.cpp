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

#define ADJUST_KP 1
#define ADJUST_KI 2
#define ADJUST_KD 3

MotorDriver* Motors::motorDriverArray[MAX_MOTORS] = {NULL, NULL,NULL,NULL,NULL,NULL};
TimePassedBy motorsLoopTimer;
int adjustPIDParam = ADJUST_KP;

// default constructor
Motors::Motors()
{
	currentMotor = NULL;				// currently set motor used for interaction
	numberOfMotors = 0;					// number of motors that have been initialized
}

void Motors::setup() {
	wristMotor.setup(0,HERKULEX_SERVO_BAUD_RATE);
	motorDriverArray[0] = &wristMotor;
	numberOfMotors = 1;
}

MotorDriver* Motors::getMotor(int motorNumber) {
	return motorDriverArray[motorNumber];
}

void Motors::printMenuHelp() {
	Serial.println(F("MotorDriver Legs"));
	Serial.println(F("0		  - consider all motors"));
	Serial.println(F("1..6    - consider motor"));
	Serial.println(F("*/'     - amend nullposition"));
	
	Serial.println(F("+/-     - adjust PID param"));
	Serial.println(F("p/i/d   - set PID tuning param"));

	Serial.println(F("h       - help"));
	Serial.println(F("esc     - exit"));
	
	for (int i = 0;i< numberOfMotors;i++)
		memory.persistentMem.motorConfig[i].println();
}



void Motors::interactiveLoop() {
	while (true)  {
		wdt_reset();
		loop();
		if (Serial.available()) {
			static char inputChar;
			inputChar = Serial.read();
			switch (inputChar) {
				case '0':
					currentMotor = NULL;
					break;
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
					currentMotor = getMotor(inputChar-'1');
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
					Serial.println(F("adjusting Kd"));
					adjustPIDParam = ADJUST_KD;
					break;
				case '+':
				case '-':
					if (currentMotor != NULL) {
						float adjust = (inputChar=='+')?0.1:-0.1;

						switch (adjustPIDParam){
							case ADJUST_KP:
								memory.persistentMem.motorConfig[currentMotor->getMotorNumber()].pivKp +=adjust;
								break;
							case ADJUST_KI:
								memory.persistentMem.motorConfig[currentMotor->getMotorNumber()].pivKi +=adjust;
								break;
							case ADJUST_KD:
								memory.persistentMem.motorConfig[currentMotor->getMotorNumber()].pivKd +=adjust;
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
					return;
				default:
					break;
			} // switch
		} // if (Serial.available())
	} // while true
}

void Motors::loop() {
	static int16_t oldAdcValue = MAX_INT_16;
	if (currentMotor != NULL) {
		if (motorKnobTimer.isDue_ms(MOTOR_KNOB_SAMPLE_RATE)) {
			// fetch value of potentiometer, returns 0..1024 representing 0..2.56V
			int16_t adcValue = analogRead(MOTOR_KNOB_PIN);
			// compare to previous adcValue
			if (oldAdcValue == MAX_INT_16)
				oldAdcValue = adcValue;
			int32_t incrementAdcValue = adcValue - oldAdcValue;
			oldAdcValue = adcValue;
			// compute angle out of adcDiff, potentiometer turns between 0°..270°
			float angle = float(incrementAdcValue*270)/1024.0;
			// turn to defined angle according to the predefined sample rate
			currentMotor->setRawAngle(angle,MOTOR_KNOB_SAMPLE_RATE);
		}
	} else
		oldAdcValue = MAX_INT_16;
	
	if (currentMotor == NULL) {
		if (motorsLoopTimer.isDue_ms(MOTOR_SAMPLE_RATE)) {
			for (int i = 0;i<numberOfMotors;i++) {
				MotorDriver* motorDriver = motorDriverArray[i];
				motorDriver->loop();
			}		
		}
	}
}
