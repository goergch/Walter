#include <avr/wdt.h>
#include "config.h"
#include <Arduino.h>
#include "PatternBlinker.h"
#include <AccelStepper.h>


int LED_PIN = 13;

static uint8_t IdlePattern[2]  =   { 0b10000000,0b00000000,};				 // boring
static uint8_t DefaultPattern[3] = { 0b11001000,0b00001100,0b10000000};	     // nice!
static uint8_t LEDOnPattern[1]  =  { 0b11111111 };
static uint8_t LEDOffPattern[1] =  { 0b00000000 };

PatternBlinker ledBlinker(LED_PIN,100);

HardwareSerial* cmdSerial = &Serial1;
HardwareSerial* logSerial = &Serial2;


AccelStepper stepper;

void forward(void* obj) {

}
void backward(void* obj) {

}

// the setup routine runs once when you press reset:
void setup() {
	// let the watchdog restart if stuck longer than 4S
	wdt_enable(WDTO_2S);

	// nice blinking pattern
	ledBlinker.set(DefaultPattern,sizeof(DefaultPattern));

	// establish serial output and say hello
	cmdSerial->begin(CONNECTION_BAUD_RATE);
	cmdSerial->println("WALTER");

	// establish logging output
	logSerial->begin(CONNECTION_BAUD_RATE);
	logSerial->println("--- WALTER's logging ---");

	stepper.setup(NULL,forward,backward);
	stepper.setMaxSpeed(1000);
}

// the loop routine runs over and over again forever:
void loop() {
	ledBlinker.loop(millis());	// blink
}
