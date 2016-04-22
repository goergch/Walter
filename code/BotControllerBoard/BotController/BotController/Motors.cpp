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

MotorDriver* Motors::motorDriverArray[MAX_MOTORS] = {NULL, NULL,NULL,NULL,NULL,NULL};


// default constructor
Motors::Motors()
{
	currentMotor = NULL;				// currently set motor used for interaction
	motorKnobOn = false;				// true if motorknob mode is on
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
	Serial.print(F("p		  - toggle motor knob "));
	Serial.println(motorKnobOn?F("on"):F("off"));

	Serial.println(F("+/-     - set angle"));

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
				case '+':
				case '-':
					currentMotor->setAngle(currentMotor->getAngle() + ((inputChar=='+')?1:-1),100);
					currentMotor->println();

					break;
				case 'p':
					motorKnobOn = !motorKnobOn;
					Serial.print(F("motorKnob is "));
					if (motorKnobOn)
						Serial.println(F("on"));
					else
						Serial.println(F("off"));

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
	if (motorKnobOn) {
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
			currentMotor->setAngle(angle,MOTOR_KNOB_SAMPLE_RATE);
		}
	} else
		oldAdcValue = MAX_INT_16;
	
	if (currentMotor == NULL) {
		// compute duration since last call of loop
		static uint32_t lastCall = 0;
		uint32_t loopDuration = millis();
		if (lastCall == 0) 
			lastCall = loopDuration;
		else
			loopDuration -= lastCall;

		for (int i = 0;i<numberOfMotors;i++) {
			MotorDriver* motorDriver = motorDriverArray[i];
			motorDriver->loop();
		}		
	}
}
