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
#include "I2CPortScanner.h"
#include "RotaryEncoder.h"
#include "PatternBlinker.h"

Motors motors;
// RotaryEncoder encoder;

extern BotMemory botMemory;
bool mainInteractive = true;

TimePassedBy elTimer;

// static uint8_t StartUp[4]  = { 0b10101010,0b10101010,0b10101010,0b10101010,}; // boring
static uint8_t BotIdles[3] = { 0b11001000,0b00001100,0b10000000};			  // nice!

PatternBlinker ledBlinker(LED,100);

void indicateCPULoad(uint8_t cpuLoad) {
	if (cpuLoad > 100)
		cpuLoad = 100;
	uint8_t duration = 10+250L*cpuLoad;
	ledBlinker.setDuration(duration);
}

void printHelp() {
	Serial.println(F("m       - motor"));
	Serial.println(F("e       - eeprom default"));
	Serial.println(F("i       - I2C port scan"));
}
void setup() {
	// being stuck after 4s let the watchdog catch it
	wdt_enable(WDTO_4S);

	// LED is on during startup
	pinMode(LED,OUTPUT);
	digitalWrite(LED, HIGH);
	
	// shudown the I2C conflicting device
	pinMode(I2C_ADDRESS_ADDON_VDD_PIN,INPUT);
	pinMode(I2C_ADDRESS_ADDON_VDD_GND,INPUT);
	digitalWrite(I2C_ADDRESS_ADDON_VDD_PIN, LOW); 
	digitalWrite(I2C_ADDRESS_ADDON_VDD_GND, LOW);

	
	// in case anything during setup goes wrong, start with UART
	Serial.begin(CONNECTION_BAUD_RATE);

	// initialize eeprom configuration values
	memory.setup();
	
	// initialize servos, steppers and encoders
	motors.setup();
	
	// initialize the magnetic encoders
	// Start Wire object. Necessary since #define USE_WIREBEGIN_ENABLED is commented out)
	// Wire.begin();
	// encoder.setup(1);
	// encoder.setup(2);
	
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
	

/*
	if (elTimer.isDue_ms(ENCODER_SAMPLE_RATE)) {
		// fetch encoder values and tell the stepper measure
		encoder.fetchAngle(); // measure the encoder's angle
		Serial.println("angle=");
		Serial.println(encoder.getAngle());
	}
*/

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

			break;

			default:
				break;
		}
	}	
}
	