/*
 * BotController.cpp
 *
 * Created: 20.04.2016 09:47:33
 *  Author: SuperJochenAlt
 */ 

#include "Arduino.h"
#include "setup.h"
#include "MotorDriver.h"
#include "Motors.h"

#include "BotMemory.h"
#include "setup.h"
#include <HkxPosControl.h>
#include <avr/wdt.h>

Motors motors;
extern BotMemory botMemory;
bool mainInteractive = true;



void printHelp() {
	Serial.println("m       - motor");
	Serial.println("e       - eeprom default");

}
void setup() {
	// being stuck after 4s let the watchdog catch it
	wdt_enable(WDTO_4S);

	pinMode(LED,OUTPUT);
	digitalWrite(LED, HIGH);
	delay(50);
	digitalWrite(LED, LOW);
	delay(100);
	digitalWrite(LED, HIGH);
	delay(50);
	digitalWrite(LED, LOW);
	
	// in case anything during setup goes wrong, start with UART
	Serial.begin(CONNECTION_BAUD_RATE);

	// initialize eeprom configuration values
	memory.setup();
	
	// initialize wrist motor with herkulex servo
	motors.setup();
	
	Serial.println(F("Snorre"));
	
	memory.println();

	printHelp();
}


	
void loop() {
	wdt_reset();

	memory.loop(); // check if config values have to be stored in EEprom

	bool interactive = motors.interactive();
	motors.loop();	
	if (interactive  && !motors.interactive()) {
		printHelp();
		mainInteractive = true;
	}
	
	if (mainInteractive && Serial.available()) {
		static char inputChar;
		inputChar = Serial.read();
		switch (inputChar) {
			case 'h':
				printHelp();
				break;
			case 'm':
				motors.interactive(true);
				mainInteractive = false;
				break;
			case 'e':
				BotMemory::setDefaults();
				memory.save();
				memory.println();
			break;

			default:
				break;
		}
	}	
}
	