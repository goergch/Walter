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
#include "SoftwareSerial.h"

Controller controller;

extern BotMemory botMemory;

bool mainInteractive = true;

static uint8_t IdlePattern[2]  =   { 0b10000000,0b00000000,};				 // boring
static uint8_t DefaultPattern[3] = { 0b11001000,0b00001100,0b10000000};	     // nice!
static uint8_t LEDOnPattern[1]  =  { 0b11111111 };
static uint8_t LEDOffPattern[1] =  { 0b00000000 };
	
PatternBlinker ledBlinker(LED_PIN,100);


void setLED(bool onOff) {
	// LEDs blinks a nice pattern during normal operations
	if (onOff)
		ledBlinker.set(LEDOnPattern,sizeof(LEDOnPattern));
	else
		ledBlinker.set(LEDOffPattern,sizeof(LEDOffPattern));
}

void setLEDPattern() {
	if (controller.isEnabled()) 
		ledBlinker.set(DefaultPattern,sizeof(DefaultPattern));
	else
		ledBlinker.set(IdlePattern,sizeof(IdlePattern));
}

void printHelp() {
	Serial.println(F("m       - motor"));
	Serial.println(F("e       - eeprom default"));
	Serial.println(F("i       - I2C port scan"));
}


void setInteractiveMode(bool on) {
	mainInteractive = on;
	if (on) {
		memory.println();
		printHelp();
	}
}

bool setupArms() {

	bool result = true;
	// everyone likes a blinking LED
	pinMode(LED_PIN,OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	// initialize eeprom configuration values
	memory.setup();

	// initialize I2C used for connection to magnetic encoders. Do it before motors.setup
	// where encoders are initialized
	Wire.begin();

	// initialize servos, steppers and encoders
	result = controller.setup() && result;
	return result;
}

void setup() {
	// two encoders have the same I2C address, one is switchable, since its power its connected to two pins
	// shutdown this conflicting encoder. Do this before initializing I2C
	RotaryEncoder::switchConflictingSensor(false /* = power off */);

	// first, disable power supply and enable/disable switch of all motors, especially of steppers
	// to avoid ticks during powering on.
	pinMode(POWER_SUPPLY_SERVO_PIN, OUTPUT);
	digitalWrite(POWER_SUPPLY_SERVO_PIN, LOW);

	pinMode(POWER_SUPPLY_STEPPER_PIN, OUTPUT);
	digitalWrite(POWER_SUPPLY_STEPPER_PIN, LOW);

	// Cant use controller.disable, since this requires a completed setup
	for (int i = 0;i<MAX_STEPPERS;i++) {
		pinMode(stepperSetup[i].enablePIN,OUTPUT);
		digitalWrite(stepperSetup[i].enablePIN, LOW);
	}

	// let the watchdog restart if stuck longer than 4S
	wdt_enable(WDTO_4S);

	// in case anything during setup goes wrong, start with UART
	Serial.begin(CONNECTION_BAUD_RATE);
	// logger->begin(LOGGER_BAUD_RATE);
		
	// nice blinking pattern
	setLEDPattern();

	// setup host communication
	hostComm.setup();			

	// setupArms();
	
	// Serial.println("setup finished");
	Serial.println(F("Snorre"));
	Serial.print(F(">"));
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
	
	hostComm.loop();

	/*
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
	*/
}
	