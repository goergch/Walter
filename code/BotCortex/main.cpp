#include <avr/wdt.h>
#include "config.h"
#include <Arduino.h>
#include "PatternBlinker.h"
#include <AccelStepper.h>

static uint8_t IdlePattern[2] = { 0b10000000, 0b00000000, };		// boring
static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };// nice!
static uint8_t LEDOnPattern[1] = { 0b11111111 };
static uint8_t LEDOffPattern[1] = { 0b00000000 };

PatternBlinker ledBlinker(LED_PIN, 100);
TimePassedBy 	motorKnobTimer;		// used for measuring sample rate of motor knob

HardwareSerial* cmdSerial = &Serial1;
HardwareSerial* logSerial = &Serial1;

AccelStepper stepper;
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
	logSerial->println("PIN assignment");
	logSerial->print("knob           = ");
	logSerial->println(KNOB_PIN);
	logSerial->print("Cmd     RX,TX  = (");
	logSerial->print(SERIAL_CMD_RX);
	logSerial->print(",");
	logSerial->print(SERIAL_CMD_RX);
	logSerial->println(")");
	logSerial->print("Logger  RX,TX  = (");
	logSerial->print(SERIAL_LOG_RX);
	logSerial->print(",");
	logSerial->print(SERIAL_LOG_RX);
	logSerial->println(")");
	logSerial->print("Herkulex RX,TX  = (");
	logSerial->print(HERKULEX_RX);
	logSerial->print(",");
	logSerial->print(HERKULEX_TX);
	logSerial->println(")");

	logSerial->print("Sensor0  SCl,SDA = (");
	logSerial->print(SENSOR0_SCL);
	logSerial->print(",");
	logSerial->print(SENSOR0_SDA);
	logSerial->println(")");

	logSerial->print("Sensor1  SCL,SDA = (");
	logSerial->print(SENSOR1_SCL);
	logSerial->print(",");
	logSerial->print(SENSOR1_SDA);
	logSerial->println(")");

	logSerial->print("Stepper0 En,Dir,CLK = (");
	logSerial->print(STEPPER0_EN_PIN);
	logSerial->print(",");
	logSerial->print(STEPPER0_DIR_PIN);
	logSerial->print(",");
	logSerial->print(STEPPER0_CLK_PIN);
	logSerial->println(")");
}


// the setup routine runs once when you press reset:
void setup() {
	// let the watchdog restart if stuck longer than 4S
	wdt_enable(WDTO_2S);

	// nice blinking pattern
	ledBlinker.set(DefaultPattern, sizeof(DefaultPattern));

	// establish serial output and say hello
	cmdSerial->begin(CONNECTION_BAUD_RATE);
	cmdSerial->println("WALTER");

	// establish logging output
	logSerial->begin(CONNECTION_BAUD_RATE);
	logSerial->println("--- WALTER's logging ---");

	logPinAssignment();

	pinMode(STEPPER0_CLK_PIN, OUTPUT);
	pinMode(STEPPER0_EN_PIN, OUTPUT);
	pinMode(STEPPER0_DIR_PIN, OUTPUT);
	pinMode(4, OUTPUT);

	digitalWrite(STEPPER0_CLK_PIN, HIGH);
	digitalWrite(STEPPER0_DIR_PIN, LOW);
	digitalWrite(STEPPER0_EN_PIN, LOW);

	stepper.setup(NULL, forward, backward);
	stepper.setMaxSpeed(1000000);
	stepper.setAcceleration(100000);

	int adcValue = analogRead(KNOB_PIN);
	float angle = (float(adcValue - 512) / 512.0) * (270.0 / 2.0);
	motorAngle = angle;
}

// the loop routine runs over and over again forever:
void loop() {
	stepper.run();

	ledBlinker.loop(millis());	// blink

	if (motorKnobTimer.isDue_ms(100, millis())) {
		int adcValue = analogRead(KNOB_PIN);
		float angle = (float(adcValue - 512) / 512.0) * (270.0 / 2.0);
		float angleDiff = angle - motorAngle;
		const float gearbox = 65.0/15.0;
		const float degreePerStep = 1.8;
		const float microsteps = 16;
		const float degreePerMicrostep = degreePerStep/microsteps;

		int steps = gearbox * angleDiff / degreePerMicrostep ;
		logSerial->print("angle=");
		logSerial->print(angle);
		logSerial->print("motor=");
		logSerial->print(motorAngle);
		logSerial->print("diff=");
		logSerial->print(angleDiff);
		logSerial->print("steps=");
		logSerial->print(steps);
		logSerial->println();

		stepper.move(steps);
		motorAngle += angleDiff;


		digitalWrite(STEPPER0_EN_PIN, HIGH);
/*
		static bool b = true;
		if (b) {
			digitalWrite(STEPPER0_EN_PIN, LOW);
			b = false;
		}
		else {
			digitalWrite(STEPPER0_EN_PIN, HIGH);
			b = true;
		}
*/
	}

	/*
	if (fabs(angle - lastAngle) > 2.0) {
		logSerial->print("knob=");
		logSerial->println(angle);

		// stepper.move((360.0/degreePerStep) * gearbox * microsteps * (angle-lastAngle));
		lastAngle = angle;

	}
	*/
}
