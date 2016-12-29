#include <Arduino.h>
#include <i2c_t3.h>
#include "watchdog.h"
#include "ams_as5048b.h"
#include "PatternBlinker.h"
#include <AccelStepper.h>
#include <I2CPortScanner.h>
#include <pins.h>
#include "config.h"
#include "hostCommunication.h"
#include "Controller.h"
#include "BotMemory.h"
#include "AMS_AS5048B.h"

// global variables declared in pins.h
HardwareSerial* cmdSerial = &Serial5;
HardwareSerial* logger = &Serial4;
HardwareSerial* servoComm = &Serial1;

i2c_t3* Wires[2] = { &Wire, &Wire1 };

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

void setLEDPattern() {
	if (controller.isEnabled())
		ledBlinker.set(DefaultPattern,sizeof(DefaultPattern));
	else
		ledBlinker.set(IdlePattern,sizeof(IdlePattern));
}


void checkOrResetI2CBus(int ic2no) {

	if (Wires[ic2no]->status() != I2C_WAITING) {
		logger->println();
		logger->print(F("reset IC2"));
		logger->print(ic2no);

		logger->print(F(" stat="));
		// Wires[ic2no] = new i2c_t3(ic2no);

		logger->println(Wires[ic2no]->status());
		Wires[ic2no]->resetBus();
		Wires[ic2no]->begin();

		logger->print(F(" reset. stat="));
		logger->println(Wires[ic2no]->status());
	}
}


TimePassedBy 	motorKnobTimer;		// used for measuring sample rate of motor knob
TimePassedBy 	encoderTimer;		// timer for encoder measurements

void logPinAssignment() {
	logger->println("--- pin assignment");
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

	logger->print("Wrist En,Dir,CLK    = (");
	logger->print(WRIST_EN_PIN);
	logger->print(",");
	logger->print(WRIST_DIR_PIN);
	logger->print(",");
	logger->print(WRIST_CLK_PIN);
	logger->println(")");

	logger->print("Forearm En,Dir,CLK  = (");
	logger->print(FOREARM_EN_PIN);
	logger->print(",");
	logger->print(FOREARM_DIR_PIN);
	logger->print(",");
	logger->print(FOREARM_CLK_PIN);
	logger->println(")");

	logger->print("Elbow En,Dir,CLK    = (");
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

	logger->print("Hip En,Dir,CLK      = (");
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

	// the following is necessary to start the sensors properly
	pinMode(PIN_SDA0, OUTPUT);
	digitalWrite(PIN_SDA0, HIGH); // switch LED on during setup
	pinMode(PIN_SCL0, OUTPUT);
	digitalWrite(PIN_SCL0, HIGH); // switch LED on during setup
	pinMode(PIN_SDA1, OUTPUT);
	digitalWrite(PIN_SDA1, HIGH); // switch LED on during setup
	pinMode(PIN_SCL1, OUTPUT);
	digitalWrite(PIN_SCL1, HIGH); // switch LED on during setup


	// disable power supply and enable/disable switch of all motors, especially of steppers
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

	// establish serial output and say hello
	cmdSerial->begin(CONNECTION_BAUD_RATE);
	cmdSerial->println("WALTER's Cortex");
	cmdSerial->print(F(">"));

	// establish logging output
	logger->begin(LOGGER_BAUD_RATE);
	logger->println("--- logging ---");
	logPinAssignment();

	// initialize I2C0 and I2C1
	Wires[0]->begin();
	// timeout should be enough to repeat the sensor request within one sample
	// on I2C0 we have 4 clients (encoder of upperarm, forearm, elbow, wrist)
	Wires[0]->setDefaultTimeout(ENCODER_SAMPLE_RATE*1000 / 4 /2);
	Wires[0]->setRate(I2C_BUS_RATE);

	Wires[1]->begin();
	// on I2C0 we have 3 clients  (hip encoder, LED driver, thermal printer)
	Wires[1]->setDefaultTimeout(ENCODER_SAMPLE_RATE*1000 / 1 / 2);
	Wires[1]->setRate(I2C_BUS_RATE);

	// log all available devices
	logger->println("--- I2C lines");
	doI2CPortScan(F("I2C0"),Wires[0], logger);
	doI2CPortScan(F("I2C1"),Wires[1], logger);
/*
	AMS_AS5048B sensor;
	sensor.setI2CAddress(0x40);
	sensor.begin(Wires[0]);

	float currentSensorAngle1 = sensor.angleR(U_DEG, true);
	float currentSensorAngle2 = sensor.angleR(U_DEG, true);
	cmdSerial->print("value 1");
	cmdSerial->println(currentSensorAngle1);
	cmdSerial->print("value 2");
	cmdSerial->println(currentSensorAngle2);

	sensor.addressRegW(0x01);
	sensor.setI2CAddress(0x44);
	sensor.programmeI2CAddress();
	doI2CPortScan(F("I2C0"),Wires[0], logger);
	currentSensorAngle1 = sensor.angleR(U_DEG, true);
	currentSensorAngle2 = sensor.angleR(U_DEG, true);
	cmdSerial->print("value 1 ");
	cmdSerial->println(currentSensorAngle1);
	cmdSerial->print("value 2 ");
	cmdSerial->println(currentSensorAngle2);

	Serial.print("value 1 ");
	Serial.print(currentSensorAngle1);
	Serial.print("value 2 ");
	Serial.print(currentSensorAngle2);

*/
	// initialize
	hostComm.setup();

	ledBlinker.set(DefaultPattern, sizeof(DefaultPattern));

	memory.setup();
	setWatchdogTimeout(2000);
}


void loop() {
	watchdogReset();
	uint32_t now = millis();
	ledBlinker.loop(now);    // blink
	hostComm.loop(now);
	memory.loop(now);
	controller.loop(millis());

	if (controller.isSetup()) {
		checkOrResetI2CBus(0);
		checkOrResetI2CBus(1);
	}
}
