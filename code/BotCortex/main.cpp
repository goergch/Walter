#include <Arduino.h>
#include <i2c_t3.h>
#include "watchdog.h"
#include "ams_as5048b.h"
#include "I2CPortScanner.h"
#include "PatternBlinker.h"
#include <AccelStepper.h>
#include <pins.h>
#include "config.h"

static uint8_t IdlePattern[2] = { 0b10000000, 0b00000000, };				// boring
static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!
static uint8_t LEDOnPattern[1] = { 0b11111111 };
static uint8_t LEDOffPattern[1] = { 0b00000000 };
PatternBlinker ledBlinker(LED_PIN, 100);

void setLED(bool onOff) {
	// LEDs blinks a nice pattern during normal operations
	if (onOff)
		ledBlinker.set(LEDOnPattern,sizeof(LEDOnPattern));
	else
		ledBlinker.set(LEDOffPattern,sizeof(LEDOffPattern));
}

TimePassedBy 	motorKnobTimer;		// used for measuring sample rate of motor knob
TimePassedBy 	encoderTimer;		// timer for encoder measurements

AccelStepper stepper;
AMS_AS5048B sensor;
float motorAngle  = 0;

void forward(void* obj) {
	digitalWrite(WRIST_DIR_PIN, HIGH);

	digitalWrite(WRIST_CLK_PIN, LOW);
	delayMicroseconds(PIBOT_PULSE_WIDTH_US);
	digitalWrite(WRIST_CLK_PIN, HIGH);
}

void backward(void* obj) {
	digitalWrite(WRIST_DIR_PIN, LOW);

	digitalWrite(WRIST_CLK_PIN, LOW);
	delayMicroseconds(PIBOT_PULSE_WIDTH_US);
	digitalWrite(WRIST_CLK_PIN, HIGH);
}

void logPinAssignment() {
	logger->println("PIN assignment");
	logger->print("knob                = ");
	logger->println(MOTOR_KNOB_PIN);
	logger->print("Power Stepper       = ");
	logger->println(POWER_SUPPLY_STEPPER_PIN);
	logger->print("Power servo         = ");
	logger->println(POWER_SUPPLY_SERVO_PIN);
	logger->print("LED                 = ");
	logger->println(LED_PIN);

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

	logger->print("Sensor0  SCL,SDA    = (");
	logger->print(SENSOR0_SCL);
	logger->print(",");
	logger->print(SENSOR0_SDA);
	logger->println(")");

	logger->print("Sensor1  SCL,SDA    = (");
	logger->print(SENSOR1_SCL);
	logger->print(",");
	logger->print(SENSOR1_SDA);
	logger->println(")");

	logger->print("Wrist En,Dir,CLK = (");
	logger->print(WRIST_EN_PIN);
	logger->print(",");
	logger->print(WRIST_DIR_PIN);
	logger->print(",");
	logger->print(WRIST_CLK_PIN);
	logger->println(")");

	logger->print("Forearm En,Dir,CLK = (");
	logger->print(FOREARM_EN_PIN);
	logger->print(",");
	logger->print(FOREARM_DIR_PIN);
	logger->print(",");
	logger->print(FOREARM_CLK_PIN);
	logger->println(")");

	logger->print("Elbow En,Dir,CLK = (");
	logger->print(ELBOW_EN_PIN);
	logger->print(",");
	logger->print(ELBOW_DIR_PIN);
	logger->print(",");
	logger->print(ELBOW_CLK_PIN);
	logger->println(")");

	logger->print("Upperarm En,Dir,CLK = (");
	logger->print(UPPERARM_EN_PIN);
	logger->print(",");
	logger->print(UPPERARM_DIR_PIN);
	logger->print(",");
	logger->print(UPPERARM_CLK_PIN);
	logger->println(")");

	logger->print("Hip En,Dir,CLK = (");
	logger->print(HIP_EN_PIN);
	logger->print(",");
	logger->print(HIP_DIR_PIN);
	logger->print(",");
	logger->print(HIP_CLK_PIN);
	logger->println(")");
}

// the setup routine runs once when you press reset:
void setup() {

	// until blinking led is initialized switch on the LED.
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH); // switch LED on during setup

	digitalWrite(LED_PIN, LOW); // switch LED on during setup
	ledBlinker.set(DefaultPattern, sizeof(DefaultPattern));


	// establish serial output and say hello
	cmdSerial->begin(CONNECTION_BAUD_RATE);
	cmdSerial->println("WALTER's Cortex");

	// establish logging output
	logger->begin(CONNECTION_BAUD_RATE);
	logger->println("--- logging ---");
	logPinAssignment();


	/*
	// initialize I2C0 and I2C1
	Wires[0]->begin();
	Wires[1]->begin();

	// log all available devices
	logger->println("I2C Bus0");
	doI2CPortScan(Wires[0], logger);
	logger->println("I2C Bus1");
	doI2CPortScan(Wires[1], logger);

	sensor.setI2CAddress(0x40);
	sensor.begin(Wires[0]);
	logger->print("I2C communication ");
	Wires[0]->beginTransmission(0x40);
	byte error = Wires[0]->endTransmission();
	logger->println(error);

	pinMode(WRIST_EN_PIN, OUTPUT);
	pinMode(WRIST_DIR_PIN, OUTPUT);
	pinMode(WRIST_CLK_PIN, OUTPUT);

	digitalWrite(WRIST_CLK_PIN, HIGH);
	digitalWrite(WRIST_DIR_PIN, LOW);
	digitalWrite(WRIST_EN_PIN, HIGH);
	stepper.setup(NULL, forward, backward);
	stepper.setMaxSpeed(1000000);
	stepper.setAcceleration(100000);

	int adcValue = analogRead(MOTOR_KNOB_PIN);
	float angle = (float(adcValue - 512) / 512.0) * (270.0 / 2.0);
	motorAngle = angle;

*/
	ledBlinker.set(IdlePattern, sizeof(IdlePattern));
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

void walterLoop();
void walterSetup();

// the loop routine runs over and over again forever:
float toBeAngle;
void loop() {
	// resetwatchdogReset();

	// stepper.run();

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
		int adcValue = analogRead(MOTOR_KNOB_PIN);
		toBeAngle = (float(adcValue - 512) / 512.0) * (270.0 / 2.0);
	}
}
