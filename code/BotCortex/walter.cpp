#include <Arduino.h>
#include "pins.h"
#include "watchdog.h"
#include "RotaryEncoder.h"
#include "Controller.h"
#include "GearedStepperDrive.h"
#include "BotMemory.h"
#include "HostCommunication.h"
#include "config.h"

extern Controller controller;
extern HostCommunication hostComm;

// global variables declared in pins.h
HardwareSerial* cmdSerial = &Serial5;
HardwareSerial* logger = &Serial4;
HardwareSerial* servoComm = &Serial1;

i2c_t3* Wires[2] = { &Wire, &Wire1 };


void walterSetup() {
	// let the watchdog restart if stuck longer than 1000ms
	setWatchdogTimeout(1000 /* ms */);


	// setup host communication
	hostComm.setup();

	// initialize eeprom configuration values
	memory.setup();

	// Serial.println("setup finished");
	Serial.println(F("WALTER CORTEX"));
	Serial.print(F(">"));

	logger->println(F("---- logger ------"));
}

void walterLoop() {
	uint32_t now = millis();
	controller.loop(now);	// control servos, steppers and encoders
	memory.loop(now);		// check if config values have changed and need to be stored in EEprom
	hostComm.loop(now);		// receive commands via Serial
}
