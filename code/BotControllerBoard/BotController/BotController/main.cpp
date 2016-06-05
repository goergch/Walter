/*
 * BotController.cpp
 *
 * Created: 20.04.2016 09:47:33
 *  Author: SuperJochenAlt
 */ 

#include "Arduino.h"
#include "setup.h"
#include "Actuator.h"
#include "Controller.h"

#include "BotMemory.h"
#include "setup.h"
#include <HkxPosControl.h>
#include <avr/wdt.h>
#include "I2CPortScanner.h"
#include "RotaryEncoder.h"
#include "PatternBlinker.h"

Controller motors;

extern BotMemory botMemory;
bool mainInteractive = true;

TimePassedBy elTimer;

// static uint8_t StartUp[4]  = { 0b10101010,0b10101010,0b10101010,0b10101010,}; // boring
static uint8_t BotIdles[3] = { 0b11001000,0b00001100,0b10000000};			  // nice!

PatternBlinker ledBlinker(LED,100);

void printHelp() {
	Serial.println(F("m       - motor"));
	Serial.println(F("e       - eeprom default"));
	Serial.println(F("i       - I2C port scan"));
}
void setup() {
	// being stuck after 4s let the watchdog catch it
	wdt_enable(WDTO_2S);

	// everyone likes a blinking LED
	pinMode(LED,OUTPUT);
	digitalWrite(LED, HIGH);
	
	// two encoders have the same I2C address, one is switchable, since its power its connected to two pins
	// shutdown this conflicting encoder. Do this before initializing I2C
	RotaryEncoder::switchConflictingSensor(false /* = power off */);
	
	// in case anything during setup goes wrong, start with UART
	Serial.begin(CONNECTION_BAUD_RATE);

	// initialize eeprom configuration values
	memory.setup();

	// initialize I2C used for connection to magnetic encoders. Do it before motors.setup
	// where encoders are initialized
	Wire.begin();

	// initialize servos, steppers and encoders
	motors.setup();
		
	// encoder1.setup(1);
	// encoder2.setup(2);
	
	
	Serial.println(F("Snorre"));
	memory.println();
	printHelp();

	// LEDs blinks a nice pattern during normal operations
	ledBlinker.set(BotIdles,sizeof(BotIdles));
}


void loop() {
	wdt_reset();
	
	ledBlinker.loop();

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
			case 'i':
				doI2CPortScan();
				break;
			default:
				break;
		}
	}	
}
	