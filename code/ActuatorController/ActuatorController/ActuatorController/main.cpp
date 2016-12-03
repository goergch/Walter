/*
 * BotController.cpp
 *
 * Created: 20.04.2016 09:47:33
 *  Author: JochenAlt
 */ 

#include "Arduino.h"
#include "Actuator.h"
#include "Controller.h"

#include "BotMemory.h"
#include <avr/wdt.h>
#include "RotaryEncoder.h"
#include "PatternBlinker.h"
#include "HostCommunication.h"
#include "utilities.h"

Controller controller;

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

void setup() {
	// let the watchdog restart if stuck longer than 4S
	wdt_enable(WDTO_2S);
	
	// first, disable power supply and enable/disable switch of all motors, especially of steppers
	// to avoid ticks during powering on.
	pinMode(POWER_SUPPLY_SERVO_PIN, OUTPUT);
	digitalWrite(POWER_SUPPLY_SERVO_PIN, LOW);

	pinMode(POWER_SUPPLY_STEPPER_PIN, OUTPUT);
	digitalWrite(POWER_SUPPLY_STEPPER_PIN, LOW);

	// until blinking led is initialized switch on the LED.
	pinMode(LED_PIN, OUTPUT); 
	digitalWrite(LED_PIN, HIGH);

	// two encoders have the same I2C address, one is switchable, since its power its connected to two pins
	// shutdown this conflicting encoder. Do this before initializing I2C
	RotaryEncoder::switchConflictingSensor(false /* = power off */);

	
	// pinMode(PIN_D5, OUTPUT); // TODO wozu ist das da?

	// Cant use controller.disable, since this requires a completed setup
	for (int i = 0;i<MAX_STEPPERS;i++) {
		pinMode(stepperSetup[i].enablePIN,OUTPUT);
		digitalWrite(stepperSetup[i].enablePIN, LOW);
	}

	// in case anything during setup goes wrong, start with UART
	Serial.begin(CONNECTION_BAUD_RATE);
	if (logger != &Serial) 
		((SoftwareSerial*)logger)->begin(LOGGER_BAUD_RATE);
		
	// setup host communication
	hostComm.setup();			

	// initialize eeprom configuration values
	memory.setup();

	// initialize I2C used for connection to magnetic encoders. Do it before motors.setup
	// where encoders are initialized
	Wire.begin();
		
	// Serial.println("setup finished");
	Serial.println(F("WALTER"));
	Serial.print(F(">"));

	if (logger != &Serial) {
		logger->println(F("---- logger ------"));
	}

	// nice blinking pattern
	setLEDPattern();
}


void loop() {
	wdt_reset();
	
	uint32_t now = millis();
	ledBlinker.loop(now);  // blink 
	memory.loop(now);		// check if config values have changed and need to be stored in EEprom
	controller.loop(now);	// control servos, steppers and encoders
	hostComm.loop(now);	// receive commands via Serial
}
	