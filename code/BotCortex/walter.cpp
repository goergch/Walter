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
