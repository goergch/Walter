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

#include <avr/wdt.h>

Motors motors;
extern BotMemory botMemory;

void setup() {
	// being stuck after 4s let the watchdog catch it
	wdt_enable(WDTO_4S);

	// in case anything during setup goes wrong, start with UART
	Serial.begin(CONNECTION_BAUD_RATE);
		
	// initialize eeprom configuration values
	memory.setup();
	
	// initialize wrist motor with herkulex servo
	motors.setup();
}


	
void loop() {
	memory.loop(); // check if config values have to be stored in EEprom
}
	