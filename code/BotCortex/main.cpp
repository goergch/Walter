#include "test-config.h"

#include <Arduino.h>
#include <i2c_t3.h>
#include "watchdog.h"
#include "ams_as5048b.h"
#include "I2CPortScanner.h"
#include "PatternBlinker.h"
#include <AccelStepper.h>

#include "Controller.h"

Controller controller;

// static uint8_t IdlePattern[2] = { 0b10000000, 0b00000000, };				// boring
static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!
// static uint8_t LEDOnPattern[1] = { 0b11111111 };
// static uint8_t LEDOffPattern[1] = { 0b00000000 };

PatternBlinker ledBlinker(LED_PIN, 100);
TimePassedBy 	motorKnobTimer;		// used for measuring sample rate of motor knob
TimePassedBy 	encoderTimer;		// timer for encoder measurements

HardwareSerial* cmdSerial = &Serial1;
HardwareSerial* logger 	  = &Serial1;
i2c_t3*			sensor0_i2c = &Wire;
i2c_t3*			sensor1_i2c = &Wire1;

AccelStepper stepper;
AMS_AS5048B sensor;
float motorAngle  = 0;

void forward(void* obj) {
	digitalWrite(STEPPER0_DIR_PIN, HIGH);

	digitalWrite(STEPPER0_CLK_PIN, LOW);
	delayMicroseconds(PIBOT_PULSE_WIDTH_US);
	digitalWrite(STEPPER0_CLK_PIN, HIGH);
}

void backward(void* obj) {
	digitalWrite(STEPPER0_DIR_PIN, LOW);

	digitalWrite(STEPPER0_CLK_PIN, LOW);
	delayMicroseconds(PIBOT_PULSE_WIDTH_US);
	digitalWrite(STEPPER0_CLK_PIN, HIGH);
}

void logPinAssignment() {
	logger->println("PIN assignment");
	logger->print("knob                = ");
	logger->println(KNOB_PIN);
	logger->print("Cmd     RX,TX       = (");
	logger->print(SERIAL_CMD_RX);
	logger->print(",");
	logger->print(SERIAL_CMD_RX);
	logger->println(")");
	logger->print("Logger  RX,TX       = (");
	logger->print(SERIAL_LOG_RX);
	logger->print(",");
	logger->print(SERIAL_LOG_RX);
	logger->println(")");
	logger->print("Herkulex RX,TX      = (");
	logger->print(HERKULEX_RX);
	logger->print(",");
	logger->print(HERKULEX_TX);
	logger->println(")");

	logger->print("Sensor0  SCl,SDA    = (");
	logger->print(SENSOR0_SCL);
	logger->print(",");
	logger->print(SENSOR0_SDA);
	logger->println(")");

	logger->print("Sensor1  SCL,SDA    = (");
	logger->print(SENSOR1_SCL);
	logger->print(",");
	logger->print(SENSOR1_SDA);
	logger->println(")");

	logger->print("Stepper0 En,Dir,CLK = (");
	logger->print(STEPPER0_EN_PIN);
	logger->print(",");
	logger->print(STEPPER0_DIR_PIN);
	logger->print(",");
	logger->print(STEPPER0_CLK_PIN);
	logger->println(")");
}

// the setup routine runs once when you press reset:
void setup() {

	// setWatchdogTimeout(1000 /* ms */);
	// nice blinking pattern
	ledBlinker.set(DefaultPattern, sizeof(DefaultPattern));

	// establish serial output and say hello
	cmdSerial->begin(CONNECTION_BAUD_RATE);
	cmdSerial->println("WALTER");

	// establish logging output
	logger->begin(CONNECTION_BAUD_RATE);
	logger->println("--- logging ---");

	logPinAssignment();

	// initialize I2C 0
	sensor0_i2c->begin();
	// log all available devices
	doI2CPortScan(logger);

	sensor.setI2CAddress(0x40);
	sensor.begin();
	logger->print("I2C communication ");
	sensor0_i2c->beginTransmission(0x40);
	byte error = sensor0_i2c->endTransmission();
	logger->println(error);

	pinMode(STEPPER0_EN_PIN, OUTPUT);
	pinMode(STEPPER0_DIR_PIN, OUTPUT);
	pinMode(STEPPER0_CLK_PIN, OUTPUT);

	digitalWrite(STEPPER0_CLK_PIN, HIGH);
	digitalWrite(STEPPER0_DIR_PIN, LOW);
	digitalWrite(STEPPER0_EN_PIN, HIGH);
	stepper.setup(NULL, forward, backward);
	stepper.setMaxSpeed(1000000);
	stepper.setAcceleration(100000);

	int adcValue = analogRead(KNOB_PIN);
	float angle = (float(adcValue - 512) / 512.0) * (270.0 / 2.0);
	motorAngle = angle;
}

float readAngle() {
	float rawAngle = sensor.angleR(U_DEG, true); // returns angle between 0..360
	float nulledRawAngle = rawAngle - 81.0;
	if (nulledRawAngle> 180.0)
		nulledRawAngle -= 360.0;
	if (nulledRawAngle< -180.0)
		nulledRawAngle += 360.0;
	return nulledRawAngle;
}

// the loop routine runs over and over again forever:
float toBeAngle;
void loop() {

	// resetwatchdogReset();

	stepper.run();

	ledBlinker.loop(millis());	// blink

	if (encoderTimer.isDue_ms(20, millis())) {
		float sensorAngle = readAngle();

		float angleDiff = toBeAngle - sensorAngle;
		const float gearbox = 65.0/15.0;
		const float degreePerStep = 1.8;
		const float microsteps = 16;
		const float degreePerMicrostep = degreePerStep/microsteps;

		int steps = (gearbox * angleDiff / degreePerMicrostep)/10;
		logger->print("angle=");
		logger->print(toBeAngle);
		logger->print("motor=");
		logger->print(sensorAngle);
		logger->print("diff=");
		logger->print(angleDiff);
		logger->print("steps=");
		logger->print(steps);
		logger->println();

		if (steps != 0)
			stepper.move(steps);
	}

	if (motorKnobTimer.isDue_ms(100, millis())) {
		int adcValue = analogRead(KNOB_PIN);
		toBeAngle = (float(adcValue - 512) / 512.0) * (270.0 / 2.0);
	}
}
