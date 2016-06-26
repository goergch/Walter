/*
 * BotController.cpp
 *
 * Created: 20.04.2016 09:47:33
 *  Author: JochenAlt
 */ 

#include "Arduino.h"
#include "setup.h"
#include "Actuator.h"
#include "Controller.h"

#include "BotMemory.h"
#include "setup.h"
#include <avr/wdt.h>
#include "I2CPortScanner.h"
#include "RotaryEncoder.h"
#include "PatternBlinker.h"
#include "HostCommunication.h"
Controller controller;

extern BotMemory botMemory;

bool mainInteractive = true;

TimePassedBy elTimer;

// static uint8_t StartUp[4]  = { 0b10101010,0b10101010,0b10101010,0b10101010,}; // boring
static uint8_t BotIdlesPattern[3] = { 0b11001000,0b00001100,0b10000000};			  // nice!
static uint8_t LEDOnPattern[1]  = { 0b1111111 };
static uint8_t LEDOffPattern[1] = { 0b00000000 };
	
PatternBlinker ledBlinker(LED,100);


void setLED(bool onOff) {
	// LEDs blinks a nice pattern during normal operations
	if (onOff)
		ledBlinker.set(LEDOnPattern,sizeof(LEDOnPattern));
	else
		ledBlinker.set(LEDOffPattern,sizeof(LEDOffPattern));
}

void setLEDPattern() {
	ledBlinker.set(BotIdlesPattern,sizeof(BotIdlesPattern));
}


void setInteractiveMode(bool on) {
	mainInteractive = on;
}
void printHelp() {
	Serial.println(F("m       - motor"));
	Serial.println(F("e       - eeprom default"));
	Serial.println(F("i       - I2C port scan"));
}

void setupArms() {

	// everyone likes a blinking LED
	pinMode(LED,OUTPUT);
	digitalWrite(LED, HIGH);

	// initialize eeprom configuration values
	memory.setup();

	// initialize I2C used for connection to magnetic encoders. Do it before motors.setup
	// where encoders are initialized
	Wire.begin();

	Serial.println(F("---------- start setup --------------"));
	doI2CPortScan();

	// initialize servos, steppers and encoders
	controller.setup();
	
	Serial.println(F("----------- end setup --------------"));
	memory.println();
	printHelp();
	
	setupArms();
}
void setup() {
	// two encoders have the same I2C address, one is switchable, since its power its connected to two pins
	// shutdown this conflicting encoder. Do this before initializing I2C
	RotaryEncoder::switchConflictingSensor(false /* = power off */);

	// being stuck after 4s let the watchdog catch it
	wdt_enable(WDTO_4S);

	// in case anything during setup goes wrong, start with UART
	Serial.begin(CONNECTION_BAUD_RATE);

	setLEDPattern();

	// setup host communication
	// hostComm.setup();	
}


void loop() {
	wdt_reset();
	
	ledBlinker.loop();

	memory.loop(); // check if config values have to be stored in EEprom

	bool interactive = controller.interactive();
	controller.loop();	
	
	if (interactive  && !controller.interactive()) {
		printHelp();
		mainInteractive = true;
	}
	
	// hostComm.loop();

	if (mainInteractive && Serial.available()) {
		static char inputChar;
		inputChar = Serial.read();
		switch (inputChar) {
			case 'h':
				printHelp();
				break;
			case 'm':
				controller.interactive(true);
				mainInteractive = false;
				break;
			case 'e':
				BotMemory::setDefaults();
				memory.save();
				memory.println();
				Serial.println(F("eeprom has been reset."));				
				break;
			case 'i':
				doI2CPortScan();
				break;
			default:
				break;
		}
	}	
}
	