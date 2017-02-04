#include <Arduino.h>
#include <i2c_t3.h>
#include "watchdog.h"
#include "ams_as5048b.h"
#include "PatternBlinker.h"
#include <I2CPortScanner.h>
#include "AccelStepper.h"
#include <pins.h>
#include "config.h"
#include "hostCommunication.h"
#include "Controller.h"
#include "BotMemory.h"
#include "AMS_AS5048B.h"
#include "core.h"
#include "LightsController.h"
#include "Printer.h"

// global variables declared in pins.h
HardwareSerial* cmdSerial = &Serial5; 		// UART used to communicate with Cerebellum
HardwareSerial* logger = &Serial4;			// UART used to log
HardwareSerial* servoComm = &Serial1;		// UART used to communicate with the HerkuleX servos
HardwareSerial* printerComm = &Serial6;		// UART used to control the thermal printer

// rotary encoders are connected via I2C
i2c_t3* Wires[2] = { &Wire, &Wire1 };		// we have two I2C buses due to conflicting sensor addresses

// blinking patterns for LED on teensy board.
static uint8_t IdlePattern[2] = { 0b10000000, 0b00000000, };				// boring
static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!
static uint8_t LEDOnPattern[1] = { 0b11111111 };
static uint8_t LEDOffPattern[1] = { 0b00000000 };
PatternBlinker ledBlinker(LED_PIN, 100 /* ms */); // one bit in the patterns above is active for 100ms

void setCortexBoardLED(bool onOff) {
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


// emergency method, that resets the I2C bus in case something went wrong (i.e. arbitration lost)
void resetI2CWhenNecessary(int ic2no) {
	if (Wires[ic2no]->status() != I2C_WAITING) {
		logger->println();

		switch(Wires[ic2no]->status())
		    {
		    case I2C_WAITING:  logger->print("I2C waiting, no errors\n"); break;
		    case I2C_ADDR_NAK: logger->print("Slave addr not acknowledged\n"); break;
		    case I2C_DATA_NAK: logger->print("Slave data not acknowledged\n"); break;
		    case I2C_ARB_LOST: logger->print("Bus Error: Arbitration Lost\n"); break;
		    case I2C_TIMEOUT:  logger->print("Bus Error: Time out\n"); break;
		    default:           logger->print("I2C busy\n"); break;
		}
		logger->print(ic2no);
		logger->print(F("("));
		logger->println(Wires[ic2no]->status());
		logger->print(F(")"));

		Wires[ic2no]->resetBus();
		Wires[ic2no]->begin();
		Wires[ic2no]->setDefaultTimeout(1000);
		Wires[ic2no]->setRate(I2C_BUS_RATE);

		logger->print(F(" stat="));
		logger->println(Wires[ic2no]->status());
	}
}

void logPinAssignment() {
	logger->println("--- pin assignment");
	logger->print("knob                = ");
	logger->println(MOTOR_KNOB_PIN);
	logger->print("Power Stepper       = ");
	logger->println(POWER_SUPPLY_STEPPER_PIN);
	logger->print("Power servo         = ");
	logger->println(POWER_SUPPLY_SERVO_PIN);
	logger->print("PCB LED             = ");
	logger->println(LED_PIN);
	logger->print("Reset LED           = ");
	logger->println(RESET_LED_PIN);

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

void setup() {

	// until ledBlinker is initialized, switch turn on the LED.
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	// the following is necessary to start the sensors properly
	// produce a LOW on i2c line to not confuse rotary encoders
	pinMode(PIN_SDA0, OUTPUT);
	digitalWrite(PIN_SDA0, HIGH);
	pinMode(PIN_SCL0, OUTPUT);
	digitalWrite(PIN_SCL0, HIGH);
	pinMode(PIN_SDA1, OUTPUT);
	digitalWrite(PIN_SDA1, HIGH);
	pinMode(PIN_SCL1, OUTPUT);
	digitalWrite(PIN_SCL1, HIGH);

	// LED within reset button
	pinMode(RESET_LED_PIN, OUTPUT);
	digitalWrite(RESET_LED_PIN, LOW);

	// disable power supply and enable/disable switch of all
	// motors, especially of steppers to avoid ticks during powering on.
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
	cmdSerial->begin(CORTEX_COMMAND_BAUD_RATE);
	cmdSerial->println("WALTER's Cortex");

	// establish logging output
	logger->begin(CORTEX_LOGGER_BAUD_RATE);
	logger->println("--- logging ---");

	// switch on servo to give them time to settle, while rest is initializing
	controller.switchServoPowerSupply(true); // works before being setup

	logPinAssignment();

	// lights console
	lights.setup();

	// initialize
	printer.setup();
	hostComm.setup();
	memory.setup();
	controller.setup();

	setWatchdogTimeout(2000);

	//done, start blinking
	ledBlinker.set(DefaultPattern, sizeof(DefaultPattern));

	// ready for input
	cmdSerial->print(F(">"));

	// read to be reset
	digitalWrite(RESET_LED_PIN, HIGH);
}


void loop() {
	watchdogReset();
	uint32_t now = millis();
	ledBlinker.loop(now);    	// LED on Teensy board
	hostComm.loop(now);			// wait for commands via serial interface
	memory.loop(now);			// check if something has to be written to EEPROM
	controller.loop(millis());	// run the actuators
	lights.loop(now);			// run the lights console

	if (controller.isSetup()) {
		resetI2CWhenNecessary(0);	// check if I2c bus is fine. Restart if not.
		resetI2CWhenNecessary(1);
	}
}
