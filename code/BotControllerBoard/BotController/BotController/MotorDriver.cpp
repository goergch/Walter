
/*
 * MotorDriver.cpp
 *
 * Created: 21.04.2016 11:52:07
 *  Author: JochenAlt
 */ 


#include "Arduino.h"
#include "MotorDriver.h"
#include "BotMemory.h"
#include <avr/wdt.h>
int currentMotor = 1;


MotorDriver* motorDriverArray[MAX_MOTORS] = {NULL, NULL,NULL,NULL,NULL,NULL};
	
	
void MotorDriverConfig::println() {
	Serial.print(nullAngle,1);
	Serial.println();
}

void MotorDriverConfig::setDefaults() {
	for (int i = 0;i<MAX_MOTORS;i++)
		memory.persistentMem.motorConfig[i].nullAngle = 0;
}


void  MotorDriver::setMotor(int pMotorNumber, const MotorDriver& motorDriver) {
	motorDriverArray[pMotorNumber] = (MotorDriver*)&motorDriver;
	motorNumber = pMotorNumber;
}
MotorDriver* MotorDriver::getMotor(int motorNumber) {
	return motorDriverArray[motorNumber];
}

void MotorDriver::addToNullPosition(float addToNullAngle) {
	memory.persistentMem.motorConfig[motorNumber].nullAngle += addToNullAngle;
}

float MotorDriver::getNullPosition() {
	return memory.persistentMem.motorConfig[motorNumber].nullAngle;
}


void MotorDriver::printMenuHelp() {
	Serial.println(F("MotorDriver Legs"));
	Serial.println(F("1..6    - consider motor"));
	Serial.println(F("+/-     - amend nullposition"));
	Serial.println(F("h       - help"));
	Serial.println(F("esc     - exit"));
}


void MotorDriver::menuController() {
	while (true)  {
		wdt_reset();
		
		if (Serial.available()) {
			static char inputChar;
			inputChar = Serial.read();
			switch (inputChar) {
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
					currentMotor = inputChar-'1';
					break;
				case '+':
				case '-':
					getMotor(currentMotor)->addToNullPosition((inputChar=='+')?1:-1);
					break;
				case 'h':
					printMenuHelp();
					break;
				case '0':
				case '\e':
					return;
				default:
					break;
			} // switch
		} // if (Serial.available())
	} // while true
}
